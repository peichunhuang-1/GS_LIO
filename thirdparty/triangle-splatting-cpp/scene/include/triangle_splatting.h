#pragma once

#include "camera.h"
#include "torch/script.h"
#include "triangle_model.h"
#include "diff_triangle_rasterization.h"
#include <cv_bridge/cv_bridge.h>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/camera_info.hpp>

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2/exceptions.h>
#include <tf2_eigen/tf2_eigen.hpp>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

class TriangleSplatting: public rclcpp::Node
{
public:
  TriangleSplatting();
  // training
  // void optimize(torch::Tensor &gt_image);

private:
  // class members
  float render_near;
  float render_far;

  float min_dist;
  float max_dist;
  int grid;
  float dist_threshold;

  int point_threshold;
  std::shared_ptr<Camera> camera;
  TriangleModel model;

  std::string camera_frame_id;
  std::string world_frame_id;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener;
  
  // pointcloud and image callbacks, need mutex to prevent race condition
  std::shared_ptr<std::mutex> mtx;
  pcl::PointCloud<pcl::PointXYZ> accum_points;
  void gt_image_cb(sensor_msgs::msg::Image::SharedPtr msg);
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr img_sub;
  void pointcloud_cb(sensor_msgs::msg::PointCloud2::SharedPtr msg);
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud_sub;
  // camera info callback for camera intrinsic
  void camera_info_cb(sensor_msgs::msg::CameraInfo::SharedPtr msg);
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr info_sub;
  
  // render_impl
  torch::autograd::tensor_list render_impl();

  // setting camera
  void set_camera_intrinsic(vk_PinholeCamera_SharedPtr pinhole_camera);
  void set_camera_pose(const Eigen::Quaternion<float>& q, const Eigen::Matrix<float, 3, 1>& t);
};