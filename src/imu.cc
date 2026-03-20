#include "imu.h"

namespace gs_lio
{

Imu::Imu() : estimator(), rclcpp::Node("gs_lio_imu_node"), mtx(std::make_shared<std::shared_mutex>())
{
  std::string imu_topic;
  this->declare_parameter<std::string>("imu.topic", "/livox/imu");
  this->get_parameter_or<std::string>("imu.topic", imu_topic, "/livox/imu");
  sub = this->create_subscription<sensor_msgs::msg::Imu>(imu_topic, rclcpp::QoS(200000), std::bind(&Imu::imu_cb, this, std::placeholders::_1));
  std::vector<double> noise;
  this->declare_parameter<std::vector<double>>("imu.noise", {0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.001, 0.001, 0.001, 0.001, 0.001, 0.001});
  this->get_parameter_or<std::vector<double>>("imu.noise", noise, {0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.001, 0.001, 0.001, 0.001, 0.001, 0.001});
  measure_noise = Eigen::Vector<double, 12>(noise.data()).cast<scalar_t>();
}

Imu::~Imu()
{
  sub.reset();
  buffer.clear();
}

void Imu::imu_cb(const sensor_msgs::msg::Imu::SharedPtr msg)
{
  std::unique_lock<std::shared_mutex> lock(*mtx);
  if (buffer.size() >= MAX_IMU_BUFFER_SIZE)
  {
    buffer.pop_front();
  }
  buffer.push_back(msg);
  cv.notify_one();
}

bool Imu::forward(const stamp_t &tailstamp) 
{
  std::unique_lock<std::shared_mutex> lock(*mtx);
  /*
  no forward, just clear imu buffer to prevent overflow, 
  and set state timestamp to latest imu measurement timestamp if buffer is not empty
  */
  if (tailstamp < 0) 
  {
    if (!buffer.empty())
    {
      state_t state = get_state();
      state.set_timestamp(stamp_to_sec(buffer.back()->header.stamp));
      set_state(state);
    }
    buffer.clear();
    return false;
  }
  /*
  forward state to tailstamp and return true, 
  or buffer.front()->header.stamp if tailstamp is later than buffer.front()->header.stamp and return false.
  */
  if (buffer.empty()) return false;
  bool ret = stamp_to_sec(buffer.front()->header.stamp) < tailstamp? false: true;
  set_state(forward_impl(get_state(), buffer.front(), tailstamp));
  buffer.pop_front();
  return ret;
}

state_t Imu::forward_impl(const state_t & state, sensor_msgs::msg::Imu::ConstSharedPtr msg, const stamp_t &tailstamp)
{
  double dt = tailstamp - state.get_timestamp();
  if (dt < 0) throw std::runtime_error("forward time difference cannot be nagtive");
  vector3_t accel_tail(msg->linear_acceleration.x, msg->linear_acceleration.y, msg->linear_acceleration.z);
  vector3_t ang_vel_tail(msg->angular_velocity.x, msg->angular_velocity.y, msg->angular_velocity.z);
  vector3_t mean_accel = 0.5 * (accel_tail + state.get_imu_acceleration()) * GRAVITY_CONSTANT - state.get_linear_bias();
  vector3_t mean_ang_vel = 0.5 * (ang_vel_tail + state.get_imu_angular_velocity()) - state.get_angular_bias();
  // propagate state
  matrix3_t mean_accel_skew = Sophus::SO3<scalar_t>::hat(mean_accel);
  matrix_t Fx = matrix_t::Identity(18, 18);
  Fx.block<3, 3>(0, 0) = so3_t::exp(-mean_ang_vel * dt).matrix();
  Fx.block<3, 3>(0, 9) = -matrix3_t::Identity() * dt;
  Fx.block<3, 3>(3, 6) = matrix3_t::Identity() * dt;
  Fx.block<3, 3>(6, 0) = -state.get_rotation().matrix() * mean_accel_skew * dt;
  Fx.block<3, 3>(6, 12) = -state.get_rotation().matrix() * dt;
  Fx.block<3, 3>(6, 15) = matrix3_t::Identity() * dt;

  matrix_t W = matrix_t::Zero(18, 18);
  W.block<3, 3>(0, 0).diagonal() = measure_noise.segment<3>(0) * dt * dt;
  W.block<3, 3>(6, 6) = state.get_rotation().matrix() * measure_noise.segment<3>(3).asDiagonal() * state.get_rotation().matrix().transpose() * dt * dt;
  W.block<3, 3>(9, 9).diagonal() = measure_noise.segment<3>(6) * dt * dt;
  W.block<3, 3>(12, 12).diagonal() = measure_noise.segment<3>(9) * dt * dt;

  auto tail_covariance = Fx * state.get_covariance() * Fx.transpose() + W;
  so3_t dR = so3_t::exp(mean_ang_vel * dt);
  vector3_t accel_world = state.get_rotation().matrix() * mean_accel + state.get_gravity();

  so3_t tail_rotation = state.get_rotation() * dR;
  vector3_t tail_translation = state.get_translation() + state.get_linear_velocity() * dt + 0.5 * accel_world * dt * dt;
  vector3_t tail_linear_velocity = state.get_linear_velocity() + accel_world * dt;
  auto state_forwarded = state_t(tailstamp, 
                                 tail_rotation, 
                                 tail_translation, 
                                 tail_linear_velocity, 
                                 state.get_angular_bias(), 
                                 state.get_linear_bias(), 
                                 state.get_gravity(), 
                                 tail_covariance);
  state_forwarded.set_imu_acceleration(accel_tail);
  state_forwarded.set_imu_angular_velocity(ang_vel_tail);
  return state_forwarded;
}

void Imu::reset(const state_t &state)
{
  std::unique_lock<std::shared_mutex> lock(*mtx);
  buffer.clear();
  estimator::reset(state);
}

}