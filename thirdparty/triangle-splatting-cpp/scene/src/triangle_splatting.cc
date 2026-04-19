#include "triangle_splatting.h"

TriangleSplatting::TriangleSplatting() : rclcpp::Node("triangle_splatting")
{
  mtx = std::make_shared<std::mutex>();
  std::string camera_ns;
  this->declare_parameter<std::string>("triangle_splatting.gt_image_ns", "/hk_camera_DA7027591");
  this->get_parameter_or<std::string>("triangle_splatting.gt_image_ns", camera_ns, "/hk_camera_DA7027591");
  img_sub = this->create_subscription<sensor_msgs::msg::Image>(camera_ns+"/rgb", 
                                                               rclcpp::QoS(200000), 
                                                               std::bind(&TriangleSplatting::gt_image_cb, this, std::placeholders::_1));
  info_sub = this->create_subscription<sensor_msgs::msg::CameraInfo>(camera_ns+"/camera_info", 
                                                                     rclcpp::QoS(200000), 
                                                                     std::bind(&TriangleSplatting::camera_info_cb, this, std::placeholders::_1));
  std::string pointcloud_topic;
  this->declare_parameter<std::string>("triangle_splatting.pointcloud_topic", "lio/pointcloud");
  this->get_parameter_or<std::string>("triangle_splatting.pointcloud_topic", pointcloud_topic, "lio/pointcloud");
  pointcloud_sub = this->create_subscription<sensor_msgs::msg::PointCloud2>(pointcloud_topic,
                                                                            rclcpp::QoS(200000),
                                                                            std::bind(&TriangleSplatting::pointcloud_cb, this, std::placeholders::_1));
  camera = nullptr;
  tf_buffer = std::make_shared<tf2_ros::Buffer>(this->get_clock());
  tf_listener = std::make_shared<tf2_ros::TransformListener>(*tf_buffer);

  this->declare_parameter<std::string>("triangle_splatting.camera_link", "camera_link");
  this->get_parameter_or<std::string>("triangle_splatting.camera_link", camera_frame_id, "camera_link");
  this->declare_parameter<std::string>("triangle_splatting.world_link", "world");
  this->get_parameter_or<std::string>("triangle_splatting.world_link", world_frame_id, "world");

  this->declare_parameter<float>("triangle_splatting.min_dist", 0.1f);
  this->get_parameter_or<float>("triangle_splatting.min_dist", min_dist, 0.1f);
  this->declare_parameter<float>("triangle_splatting.max_dist", 20.0f);
  this->get_parameter_or<float>("triangle_splatting.max_dist", max_dist, 20.0f);
  this->declare_parameter<int>("triangle_splatting.grid", 5);
  this->get_parameter_or<int>("triangle_splatting.grid", grid, 5);
  this->declare_parameter<float>("triangle_splatting.dist_threshold", 0.05f);
  this->get_parameter_or<float>("triangle_splatting.dist_threshold", dist_threshold, 0.05f);

  this->declare_parameter<float>("triangle_splatting.render_near", 0.01);
  this->get_parameter_or<float>("triangle_splatting.render_near", render_near, render_near);
  this->declare_parameter<float>("triangle_splatting.render_far", 100.0);
  this->get_parameter_or<float>("triangle_splatting.render_far", render_far, render_far);
  this->declare_parameter<int>("triangle_splatting.point_threshold", 1000000);
  this->get_parameter_or<int>("triangle_splatting.point_threshold", point_threshold, point_threshold);
  
  std::string combined_img_topic;
  this->declare_parameter<std::string>("triangle_splatting.combined_img_topic", "triangle_splatting/combined");
  this->get_parameter_or<std::string>("triangle_splatting.combined_img_topic", combined_img_topic, "triangle_splatting/combined");
  combined_img_pub = this->create_publisher<sensor_msgs::msg::Image>(combined_img_topic, rclcpp::QoS(10));
}

void TriangleSplatting::set_camera_intrinsic(vk_PinholeCamera_SharedPtr pinhole_camera)
{
  if (camera == nullptr) { // only set at first or camera being reset to nullptr
    camera = std::make_shared<Camera>(pinhole_camera, render_near, render_far);
  }
}

void TriangleSplatting::set_camera_pose(const Eigen::Quaternion<float>& q, const Eigen::Matrix<float, 3, 1>& t)
{
  if (camera != nullptr) {
    camera->setPose(q, t);
  }
}

void TriangleSplatting::gt_image_cb(sensor_msgs::msg::Image::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(*mtx);
  if (accum_points.size() < point_threshold || camera == nullptr) return;
  if (accum_points.size() < point_threshold || camera == nullptr) return;
  cv_bridge::CvImagePtr cv_ptr;
  cv_ptr = cv_bridge::toCvCopy(msg, msg->encoding);
  cv::Mat image = cv_ptr->image;
  
  bool success = tf_buffer->canTransform(
    world_frame_id,         // target
    camera_frame_id,        // source
    msg->header.stamp,
    tf2::durationFromSec(3.0)
  );

  if (!success) throw std::runtime_error("Timeout waiting camera tf...");
  auto T_camera_world = tf_buffer->lookupTransform(
    world_frame_id,
    camera_frame_id,
    msg->header.stamp
  );
  const auto &t = T_camera_world.transform.translation;
  const auto &q = T_camera_world.transform.rotation;

  Eigen::Vector3f t_vec = Eigen::Vector3d(t.x, t.y, t.z).cast<float>();
  Eigen::Quaternionf q_eigen = Eigen::Quaterniond(q.w, q.x, q.y, q.z).cast<float>();

  t_vec = -q_eigen.toRotationMatrix().transpose() * t_vec;
  set_camera_pose(q_eigen, t_vec);

  success = model.start_from_pcd_and_keyframe(accum_points, *camera, image, min_dist, max_dist, grid, dist_threshold);
  accum_points.points.clear();
  if (!success) return;
  auto ret = render_impl();
  auto image_tensor = ret[0];
  image_tensor = image_tensor.detach().to(torch::kCPU).mul(255).clamp(0, 255).to(torch::kUInt8).permute({1, 2, 0}).contiguous();
  cv::Mat img(image_tensor.size(0), image_tensor.size(1), CV_8UC3, image_tensor.data_ptr());
  
  cv::Mat original_bgr;
  if (image.channels() == 3 && image.type() == CV_8UC3) {
    if (msg->encoding == "rgb8") {
      cv::cvtColor(image, original_bgr, cv::COLOR_RGB2BGR);
    } else {
      original_bgr = image.clone();
    }
  } else {
    original_bgr = image.clone();
  }
  
  // combine images vertically (original on top, rendered on bottom)
  cv::Mat combined(original_bgr.rows + img.rows, original_bgr.cols, CV_8UC3);
  original_bgr.copyTo(combined(cv::Rect(0, 0, original_bgr.cols, original_bgr.rows)));
  img.copyTo(combined(cv::Rect(0, original_bgr.rows, img.cols, img.rows)));
  
  // publish combined image
  auto combined_msg = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", combined).toImageMsg();
  combined_msg->header.stamp = msg->header.stamp;
  combined_msg->header.frame_id = camera_frame_id;
  combined_img_pub->publish(*combined_msg);
}

void TriangleSplatting::camera_info_cb(sensor_msgs::msg::CameraInfo::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(*mtx);
  if (msg->distortion_model != "plumb_bob") {
    throw std::runtime_error("Unsupported camera model");
  }
  auto pinhole_model = std::make_shared<vk::PinholeCamera>(
    msg->width,
    msg->height,
    1.0,
    msg->k[0],
    msg->k[4],
    msg->k[2],
    msg->k[5],
    msg->d[0],
    msg->d[1],
    msg->d[2],
    msg->d[3]
  );
  set_camera_intrinsic(pinhole_model);
}

void TriangleSplatting::pointcloud_cb(sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(*mtx);
  pcl::PointCloud<pcl::PointXYZ> point_cloud;
  pcl::fromROSMsg(*msg, point_cloud);
  
  accum_points.points.insert(
    accum_points.points.end(),
    point_cloud.points.begin(),
    point_cloud.points.end()
  );
}

torch::autograd::tensor_list TriangleSplatting::render_impl()
{
  auto means2D = torch::zeros_like(model.get_triangle_points().index({"...", 0, "..."}).squeeze()).set_requires_grad(true).to(torch::kCUDA);
  auto scaling = torch::zeros_like(means2D.index({"...", 0})).set_requires_grad(true).to(torch::kCUDA).detach();  
  auto density_factor = torch::zeros_like(scaling).set_requires_grad(true).to(torch::kCUDA).detach();
  means2D.retain_grad();

  auto rasterize_settings = TriangleSplattingSettings {
    camera->height(),
    camera->width(),
    tanf(camera->fovx() * 0.5),
    tanf(camera->fovy() * 0.5),
    torch::tensor({0.f, 0.f, 0.f}, torch::kFloat32).to(torch::kCUDA),
    camera->getViewMatrix(),
    camera->getFullViewMatrix(),
    model.sh_degree(),
    camera->get_campos(),
    false
  };

  auto rasterizer = TriangleRasterizer(rasterize_settings);

  auto opacity = model.get_opacity();
  auto sigma = model.get_sigma();
  auto triangles_points = model.get_triangles_points_flatten();
  auto num_points_per_triangle = model.get_num_points_per_triangle();
  auto cumsum_of_points_per_triangle = model.get_cumsum_of_points_per_triangle();
  auto number_of_points = model.get_number_of_points();
  auto shs = model.get_sh();
  auto mask = ((torch::sigmoid(model.get_opacity()) > 0.01).to(torch::kFloat32) - torch::sigmoid(model.get_opacity())).detach() + torch::sigmoid(model.get_opacity());
  opacity = opacity * mask;
  return rasterizer.forward(
    triangles_points,
    sigma,
    num_points_per_triangle,
    cumsum_of_points_per_triangle,
    number_of_points,
    opacity,
    means2D,
    scaling,
    density_factor,
    shs,
    torch::Tensor()
  );
}


int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);
  auto triangle_node = std::make_shared<TriangleSplatting>();
  rclcpp::spin(triangle_node);
  rclcpp::shutdown();
  return 0;
}