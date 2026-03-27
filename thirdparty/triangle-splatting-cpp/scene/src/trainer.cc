#include "render.h"
#include "loss.h"

void save_frame(const torch::Tensor& img, const std::string& path)
{
    torch::Tensor cpu = img.detach().contiguous().to(torch::kCPU);

    int h = cpu.size(0);
    int w = cpu.size(1);
    int c = cpu.size(2);

    std::ofstream f(path, std::ios::binary);

    // header
    f.write((char*)&w, sizeof(int));
    f.write((char*)&h, sizeof(int));
    f.write((char*)&c, sizeof(int));

    // data
    f.write((char*)cpu.data_ptr<float>(), w*h*c*sizeof(float));
}

torch::Tensor load_frame(const std::string& path)
{
    std::ifstream f(path, std::ios::binary);
    if(!f)
        throw std::runtime_error("cannot open file");

    int w, h, c;
    f.read((char*)&w, sizeof(int));
    f.read((char*)&h, sizeof(int));
    f.read((char*)&c, sizeof(int));

    std::vector<float> buffer(w*h*c);
    f.read((char*)buffer.data(), buffer.size()*sizeof(float));

    if(!f)
        throw std::runtime_error("file truncated");

    auto tensor = torch::from_blob(
        buffer.data(),
        {h, w, c},
        torch::kFloat32
    ).clone(); // IMPORTANT (buffer will free)

    return tensor;
}

int main(int argc, char* argv[])
{
  if (argc < 2) {return 0;}
  std::string filepath = std::string(argv[1]);
  TriangleModel model;
  // model.load(filepath);
  std::vector<char> file_bytes = get_bytes_from_file("../point_cloud_state_dict.pt");
  c10::Dict<c10::IValue, c10::IValue> point_cloud_state_dict = torch::pickle_load(file_bytes).toGenericDict();
  // 2. Extract triangle points to get the shape
  at::Tensor triangles_raw = point_cloud_state_dict.at("triangles_points").toTensor();
  at::Tensor features_dc = point_cloud_state_dict.at("features_dc").toTensor();
  // model.extend_from_pcd(triangles_raw, features_dc, 3);

  std::shared_ptr<vk::PinholeCamera> pinhole = std::make_shared<vk::PinholeCamera>(
    1280, 
    1024, 
    1.0, 
    1311.567600, 
    1311.985080, 
    658.357890, 
    482.222515,     
    -0.084877, 
    0.123077, 
    0.000066, 
    -0.000105
  );
  Camera cam(pinhole, 0.1, 1e3);
  cam.setPose(Eigen::Quaternionf(0.2659324, 0.225115, 0.6910962, -0.6332371), Eigen::Vector3f(-1.18699, 0.35, 0.0323));
  torch::Tensor image_tensor;
  auto ret = render(cam, model);
  image_tensor = ret[0];
  image_tensor = image_tensor.detach().cpu().permute({1, 2, 0}).contiguous();
  save_frame(image_tensor, "output1.bin");

  auto gt_image = load_frame("gt.bin");
  gt_image = gt_image.permute({2,0,1});     // HWC -> CHW
  gt_image = gt_image.unsqueeze(0);        // CHW -> 1CHW
  gt_image = gt_image.to(torch::kCUDA).to(torch::kFloat32);
  for (int i = 0; i < 10; i++)
  {
    auto ret = render(cam, model);

    image_tensor = ret[0];
    auto pixel_loss = loss_utils::l1_loss(image_tensor, gt_image);
    pixel_loss.backward();
    {
      torch::NoGradGuard no_grad;
      model.step(); 
    }
    if (i % 10 == 0) {
      std::cout << "[Iter " << i << "] Loss = "
                << pixel_loss.item<float>() << std::endl;
    }
    // image_tensor = image_tensor.detach().cpu().permute({1, 2, 0}).contiguous();
    // save_frame(image_tensor, "output.bin");
  }
  image_tensor = image_tensor.detach().cpu().permute({1, 2, 0}).contiguous();
  save_frame(image_tensor, "output.bin");
  return 0;
}