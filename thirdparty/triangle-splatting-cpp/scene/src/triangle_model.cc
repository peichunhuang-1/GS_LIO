#include "triangle_model.h"

std::vector<char> get_bytes_from_file(const std::string& filename) {
    std::ifstream input(filename, std::ios::binary);
    std::vector<char> bytes(
        (std::istreambuf_iterator<char>(input)),
        (std::istreambuf_iterator<char>())
    );
    input.close();
    return bytes;
}

TriangleModel::TriangleModel() 
{
}

void TriangleModel::setup_optimizer(double lr)
{
  std::vector<torch::optim::OptimizerParamGroup> optimizer_params_groups;
  optimizer_params_groups.reserve(6);
  optimizer_params_groups.push_back(torch::optim::OptimizerParamGroup({_features_dc}, std::make_unique<torch::optim::AdamOptions>(lr)));
  optimizer_params_groups.push_back(torch::optim::OptimizerParamGroup({_features_rest}, std::make_unique<torch::optim::AdamOptions>(lr / 20.0)));
  optimizer_params_groups.push_back(torch::optim::OptimizerParamGroup({_opacity}, std::make_unique<torch::optim::AdamOptions>(lr)));
  optimizer_params_groups.push_back(torch::optim::OptimizerParamGroup({_triangles_points}, std::make_unique<torch::optim::AdamOptions>(lr)));
  optimizer_params_groups.push_back(torch::optim::OptimizerParamGroup({_sigma}, std::make_unique<torch::optim::AdamOptions>(lr)));
  optimizer_params_groups.push_back(torch::optim::OptimizerParamGroup({_mask}, std::make_unique<torch::optim::AdamOptions>(lr)));
  
  static_cast<torch::optim::AdamOptions&>(optimizer_params_groups[0].options()).eps(1e-15);
  static_cast<torch::optim::AdamOptions&>(optimizer_params_groups[1].options()).eps(1e-15);
  static_cast<torch::optim::AdamOptions&>(optimizer_params_groups[2].options()).eps(1e-15);
  static_cast<torch::optim::AdamOptions&>(optimizer_params_groups[3].options()).eps(1e-15);
  static_cast<torch::optim::AdamOptions&>(optimizer_params_groups[4].options()).eps(1e-15);
  static_cast<torch::optim::AdamOptions&>(optimizer_params_groups[5].options()).eps(1e-15);
  _optimizer = std::make_unique<torch::optim::Adam>(optimizer_params_groups, torch::optim::AdamOptions(0.f).eps(1e-15));
}

void TriangleModel::start_from_pcd_and_keyframe(const pcl::PointCloud<pcl::PointXYZ> &pcd, const Camera &camera, const cv::Mat &keyframe)
{
  std::tuple<torch::Tensor, torch::Tensor> triangles_and_features = TriangulationCUDA(
    pcd.points.size(),
    keyframe.rows, keyframe.cols,
    pcl_to_tensor(pcd),
    cv_to_tensor(keyframe),
    camera.getViewMatrix(),
    camera.getFullViewMatrix(),
    0.1,
    10.0,
    5,
    0.05
  );
}

torch::Tensor TriangleModel::cv_to_tensor(const cv::Mat &img) {
  cv::Mat img_float;
  img.convertTo(img_float, CV_32FC3, 1.0f / 255.0f);
  auto options = torch::TensorOptions().dtype(torch::kFloat32).device(torch::kCPU);
  torch::Tensor tensor = torch::from_blob(img_float.data, {img.rows, img.cols, 3}, options);
  return tensor.permute({2, 0, 1}).to(torch::kCUDA).contiguous();
}

torch::Tensor TriangleModel::pcl_to_tensor(const pcl::PointCloud<pcl::PointXYZ> &pcd)
{
  int N = pcd.size();
  torch::Tensor tensor = torch::empty({N, 3}, torch::kFloat32);

  float* ptr = tensor.data_ptr<float>();

  for (int i = 0; i < N; ++i) {
    ptr[i * 3 + 0] = pcd.points[i].x;
    ptr[i * 3 + 1] = pcd.points[i].y;
    ptr[i * 3 + 2] = pcd.points[i].z;
  }

  return tensor.to(torch::kCUDA);
}

void TriangleModel::extend_from_pcd(at::Tensor new_triangles, at::Tensor new_feature_dc, const int sh_degree)
{
  this->_triangles_points = new_triangles.to(torch::kCUDA).to(torch::kFloat32).detach().clone().set_requires_grad(true);
  this->_features_dc = new_feature_dc.to(torch::kCUDA).to(torch::kFloat32).detach().clone().set_requires_grad(true);
  int64_t num_triangles = new_triangles.size(0);  
  this->_sigma = torch::ones({num_triangles, 1}, 
                  torch::TensorOptions().dtype(torch::kFloat32).device(torch::kCUDA)) * -2.40794560865; // 0.1 after get sigma
  this->_sigma.detach_(); // Explicitly ensure it's a leaf after the multiplication
  this->_sigma.set_requires_grad(true);

  this->_features_rest = torch::zeros({num_triangles, 45, 3}, 
                                      torch::TensorOptions().dtype(torch::kFloat32).device(torch::kCUDA))  
                                      .set_requires_grad(true);
  
  this->_opacity = torch::zeros({num_triangles, 1}, 
                                torch::TensorOptions().dtype(torch::kFloat32).device(torch::kCUDA))  
                                .set_requires_grad(true); // 0.5 after get opacity
  
  this->_mask = torch::ones({num_triangles, 1},   
                           torch::TensorOptions().dtype(torch::kFloat32).device(torch::kCUDA))  
                           .set_requires_grad(true);  


  this->active_sh_degree = sh_degree;

  this->_num_points_per_triangle = torch::ones({num_triangles},   
                                               torch::TensorOptions().dtype(torch::kInt32).device(torch::kCUDA)) * 3; 
  at::Tensor padded = torch::constant_pad_nd(this->_num_points_per_triangle.detach().clone(), {1, 0}, 0);
  this->_cumsum_of_points_per_triangle = torch::cumsum(padded, 0, torch::kInt)  
                                          .slice(0, 0, -1);  
  this->_number_of_points = num_triangles;
}

void TriangleModel::load(const std::string& path) 
{
    // 1. Load the state_dict (saved via torch.save)

    std::vector<char> file_bytes = get_bytes_from_file(path + "/point_cloud_state_dict.pt");
    c10::Dict<c10::IValue, c10::IValue> point_cloud_state_dict = torch::pickle_load(file_bytes).toGenericDict();

    // 2. Extract triangle points to get the shape
    at::Tensor triangles_raw = point_cloud_state_dict.at("triangles_points").toTensor();
    int64_t max_shape = triangles_raw.size(0);
    std::cout << "Loaded " << max_shape << " triangle shapes" << std::endl;

    int64_t i = 0;
    int64_t plus = max_shape;

    // Helper lambda to mimic: [i:i+plus].to("cuda").to(float32).detach().clone().requires_grad_(true)
    auto process_tensor = [&](const std::string& key) {
        return point_cloud_state_dict.at(key).toTensor()
            .slice(0, i, i + plus)
            .to(torch::kCUDA)
            .to(torch::kFloat32)
            .detach()
            .clone()
            .set_requires_grad(true);
    };

    // 3. Assign tensors
    this->_triangles_points = process_tensor("triangles_points");
    this->_sigma = process_tensor("sigma");
    this->_features_dc = process_tensor("features_dc");
    this->_features_rest = process_tensor("features_rest");
    this->_opacity = process_tensor("opacity");
    
    // Extract non-tensor value
    this->active_sh_degree = point_cloud_state_dict.at("active_sh_degree").toInt();

    // // 4. Setup Mask
    this->_mask = torch::ones({_triangles_points.size(0), 1}, 
                               torch::TensorOptions().device(torch::kCUDA))
                               .set_requires_grad(true);

    // 5. Calculate points per triangle
    // In C++, we can use the size(1) or sizes() if it's a 3D/jagged structure
    // Since triangles_points[i] in your Python code implies indexing, 
    // we assume it's a tensor where dim 1 represents point count.
    
    // Equivalent to your loop: self._triangles_points[i].shape[0]
    // If _triangles_points is [N, P, D], then points per triangle is a constant P
    int64_t num_triangles = triangles_raw.size(0);
    std::vector<int32_t> num_points_vec;
    num_points_vec.reserve(num_triangles);

    // Replicate the Python loop: for i in range(...): append(shape[0])
    for (int64_t i = 0; i < num_triangles; ++i) {
        // Get the size of the 0-th dimension for each individual tensor in the list
        num_points_vec.push_back(static_cast<int32_t>(triangles_raw[i].size(0)));
    }

    // Convert the std::vector to a CUDA tensor
    // We use torch::from_blob to wrap the vector, then clone it to move to GPU
    auto options = torch::TensorOptions().dtype(torch::kInt32).device(torch::kCPU);
    at::Tensor tensor_num_points_per_triangle = torch::from_blob(num_points_vec.data(), 
                                                    {num_triangles}, 
                                                    options)
                                                    .to(torch::kCUDA)
                                                    .clone();

    this->_num_points_per_triangle = tensor_num_points_per_triangle;
    // 6. Cumsum and Padding
    // Python: F.pad(tensor, (1,0), value=0)
    at::Tensor padded = torch::constant_pad_nd(tensor_num_points_per_triangle, {1, 0}, 0);
    
    // Python: torch.cumsum(... , 0)[:-1]
    this->_cumsum_of_points_per_triangle = torch::cumsum(padded, 0, torch::kInt)
                                            .slice(0, 0, -1);
                                            
    this->_number_of_points = _triangles_points.size(0);
}