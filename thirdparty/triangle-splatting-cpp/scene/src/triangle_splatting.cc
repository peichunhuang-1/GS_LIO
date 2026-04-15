#include "triangle_splatting.h"

TriangleSplatting::TriangleSplatting(const float near, const float far) : render_near(near), render_far(far), rclcpp::Node("triangle_splatting")
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
  cv_bridge::CvImagePtr cv_ptr;
  cv_ptr = cv_bridge::toCvCopy(msg, msg->encoding);
  cv::Mat image = cv_ptr->image;
  
  bool success = tf_buffer->canTransform(
    world_frame_id,       // target
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

  model.start_from_pcd_and_keyframe(accum_points, *camera, image);
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


int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);
  auto triangle_node = std::make_shared<TriangleSplatting>();
  rclcpp::spin(triangle_node);
  rclcpp::shutdown();
  return 0;
}