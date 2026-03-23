#ifndef GS_LIO_IMU_H
#define GS_LIO_IMU_H

#include "estimator.h"
#include "sensor_msgs/msg/imu.hpp"

#define MAX_IMU_BUFFER_SIZE 6000


namespace gs_lio
{
class Imu: public estimator, public rclcpp::Node
{
public:
  Imu(const std::string & name);
  ~Imu();
  stamp_t wait_imu(int timeout_ms = 200);
  bool forward(const stamp_t &tailstamp) override;
  void reset(const state_t &state) override;
private:
  void imu_cb(const sensor_msgs::msg::Imu::SharedPtr msg);
  state_t forward_impl(const state_t &state, sensor_msgs::msg::Imu::ConstSharedPtr msg, const stamp_t &tailstamp);
  std::shared_ptr<std::shared_mutex> mtx;
  std::deque<sensor_msgs::msg::Imu::ConstSharedPtr> buffer;
  std::condition_variable_any cv;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr sub;
  vector12_t measure_noise = vector12_t::Zero();
};

} // namespace gs_lio

#endif