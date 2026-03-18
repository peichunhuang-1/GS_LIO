#include "livox_plugin.h"

namespace gs_lio
{
  
Livox::Livox(rclcpp::Node &node) : Lidar(node)
{
  std::string lidar_topic;
  node.declare_parameter<std::string>("lidar.topic", "/livox/lidar");
  node.get_parameter_or<std::string>("lidar.topic", lidar_topic, "/livox/lidar");
  subscription = node.create_subscription<livox_ros_driver2::msg::CustomMsg>(
    lidar_topic, rclcpp::QoS(200000), std::bind(&Livox::cb, this, std::placeholders::_1));
}

void Livox::process_livox_msg(const livox_ros_driver2::msg::CustomMsg::SharedPtr msg)
{
  std::unique_lock<std::shared_mutex> lock(*mtx);
  int pc_size = msg->point_num;
  std::shared_ptr<pcl::PointCloud<pcl::PointXYZIT>> pointcloud = std::make_shared<pcl::PointCloud<pcl::PointXYZIT>>();
  pointcloud->points.clear();
  pointcloud->points.reserve(pc_size);
  for (int i = 1; i < pc_size; i++) {
    if (blind_zone * blind_zone > msg->points[i].x * msg->points[i].x + msg->points[i].y * msg->points[i].y + msg->points[i].z * msg->points[i].z) continue;
    pcl::PointXYZIT p;
    p.x = msg->points[i].x;
    p.y = msg->points[i].y;
    p.z = msg->points[i].z;
    p.intensity = msg->points[i].reflectivity;
    p.timestamp = static_cast<double>(msg->points[i].offset_time) / 1e9 + stamp_to_sec(msg->header.stamp);
    tailstamp = std::max(tailstamp, p.timestamp); // TODO: should check if tailstamp bigger than p.timestamp?
    if ((abs(msg->points[i].x - msg->points[i-1].x) > 1e-7) || (abs(msg->points[i].y - msg->points[i-1].y) > 1e-7) ||
      (abs(msg->points[i].z - msg->points[i-1].z) > 1e-7)) {
      pointcloud->points.push_back(p);
    }
  }
  tailstamps.push(tailstamp);
  raw_pointcloud_buffer.push(pointcloud);
  cv.notify_all();
}

void Livox::cb(const livox_ros_driver2::msg::CustomMsg::SharedPtr msg)
{
  process_livox_msg(msg);
}

}