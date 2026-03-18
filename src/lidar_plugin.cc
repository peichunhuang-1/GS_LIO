#include "lidar_plugin.h"

namespace gs_lio
{

Lidar::Lidar(rclcpp::Node &node)
{
  mtx = std::make_shared<std::shared_mutex>();
  node.declare_parameter<int>("lidar.filter_ratio", filter_ratio);
  node.get_parameter_or<int>("lidar.filter_ratio", filter_ratio, 1);
  node.declare_parameter<scalar_t>("lidar.blind_zone", blind_zone);
  node.get_parameter_or<scalar_t>("lidar.blind_zone", blind_zone, 0.1);
  node.declare_parameter<scalar_t>("lidar.range_covariance", range_covariance);
  node.get_parameter_or<scalar_t>("lidar.range_covariance", range_covariance, 0.01);
  node.declare_parameter<scalar_t>("lidar.angle_covariance", angle_covariance);
  node.get_parameter_or<scalar_t>("lidar.angle_covariance", angle_covariance, 0.01);
}

stamp_t Lidar::wait_available(const int &timeout_ms) {
  std::shared_lock<std::shared_mutex> lock(*mtx);
  if (!tailstamps.empty()) return tailstamps.front();
  if (timeout_ms >= 0)
  {
    auto status = cv.wait_for(lock, std::chrono::milliseconds(timeout_ms));
    if (status == std::cv_status::timeout) {
      return -1;
    }
    else return tailstamps.empty()? -1: tailstamps.front();
  }  
  cv.wait(lock);
  return tailstamps.empty()? -1: tailstamps.front();
}

std::shared_ptr<pcl::PointCloud<pcl::PointXYZIT>> Lidar::get_pointcloud() {
  std::unique_lock<std::shared_mutex> lock(*mtx);
  if (tailstamps.empty()) throw std::runtime_error("lidar is not update or data already retreived, the pointcloud data may be undefined");
  auto pointcloud = raw_pointcloud_buffer.front();
  raw_pointcloud_buffer.pop();
  tailstamps.pop();
  return pointcloud;
}

matrix3_t Lidar::point_covariance(const vector3_t &point) {
  matrix3_t cov = matrix3_t::Zero();
  scalar_t r = std::sqrt(point.x() * point.x() + point.y() * point.y() + point.z() * point.z());
  if (r < 1e-6) r = 1e-6;
  matrix2_t direction_var;
  direction_var << pow(sin(angle_covariance), 2), 0, 0, pow(sin(angle_covariance), 2);
  vector3_t direction = point.normalized();
  matrix3_t direction_hat;
  direction_hat << Sophus::SO3<scalar_t>::hat(direction);
  vector3_t base_1 = vector3_t(direction(2), direction(2), -direction(0) - direction(1)).normalized();
  vector3_t base_2 = direction.cross(base_1).normalized();
  matrix32_t N; // 3 x 2 matrix, N = [base_1, base_2]
  N << base_1(0), base_2(0), base_1(1), base_2(1), base_1(2), base_2(2);
  matrix32_t A = r * direction_hat * N;
  cov = direction * range_covariance * range_covariance * direction.transpose() + A * direction_var * A.transpose();
  return cov;
}

}