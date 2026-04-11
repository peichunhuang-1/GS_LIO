#include "imu.h"

namespace gs_lio
{

Imu::Imu(const std::string & name) : estimator(), rclcpp::Node(name), mtx(std::make_shared<std::shared_mutex>())
{
  std::string imu_topic;
  this->declare_parameter<std::string>("imu.world_frame", "world");
  this->get_parameter_or<std::string>("imu.world_frame", world_frame, "world");
  this->declare_parameter<std::string>("imu.topic", "/livox/imu");
  this->get_parameter_or<std::string>("imu.topic", imu_topic, "/livox/imu");
  sub = this->create_subscription<sensor_msgs::msg::Imu>(imu_topic, rclcpp::QoS(200000), std::bind(&Imu::imu_cb, this, std::placeholders::_1));
  std::vector<double> noise;
  this->declare_parameter<std::vector<double>>("imu.noise", {0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.001, 0.001, 0.001, 0.001, 0.001, 0.001});
  this->get_parameter_or<std::vector<double>>("imu.noise", noise, {0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.001, 0.001, 0.001, 0.001, 0.001, 0.001});
  measure_noise = Eigen::Vector<double, 12>(noise.data()).cast<scalar_t>();
  this->declare_parameter<std::string>("imu.imu_link", "imu_link");
  this->get_parameter_or<std::string>("imu.imu_link", imu_link, "imu_link");
  tf_broadcaster = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
}

Imu::~Imu()
{
  sub.reset();
  buffer.clear();
}

void Imu::publish_tf(const state_t &state)
{
  geometry_msgs::msg::TransformStamped transform_stamped;
  transform_stamped.header.stamp = rclcpp::Time(state.get_timestamp() * 1e9);
  transform_stamped.header.frame_id = world_frame;
  transform_stamped.child_frame_id = imu_link;
  transform_stamped.transform.translation.x = state.get_translation().x();
  transform_stamped.transform.translation.y = state.get_translation().y();
  transform_stamped.transform.translation.z = state.get_translation().z();
  auto q = state.get_rotation().unit_quaternion();
  transform_stamped.transform.rotation.w = q.w();
  transform_stamped.transform.rotation.x = q.x();
  transform_stamped.transform.rotation.y = q.y();
  transform_stamped.transform.rotation.z = q.z();
  tf_broadcaster->sendTransform(transform_stamped);
}

stamp_t Imu::wait_imu(int timeout_ms)
{
  std::unique_lock<std::shared_mutex> lock(*mtx);
  if (timeout_ms >= 0)
  {
    auto status = cv.wait_for(lock, std::chrono::milliseconds(timeout_ms));
    if (status == std::cv_status::timeout) {
      return -1;
    }
    else return stamp_to_sec(buffer.back()->header.stamp);
  }  
  cv.wait(lock);
  return stamp_to_sec(buffer.back()->header.stamp);
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
      auto msg = buffer.back();
      vector3_t accel_tail(msg->linear_acceleration.x, msg->linear_acceleration.y, msg->linear_acceleration.z);
      vector3_t ang_vel_tail(msg->angular_velocity.x, msg->angular_velocity.y, msg->angular_velocity.z);
      state.set_timestamp(stamp_to_sec(msg->header.stamp));
      state.set_imu_acceleration(accel_tail);
      state.set_imu_angular_velocity(ang_vel_tail);
      set_state(state);
      publish_tf(state);
    }
    buffer.clear();
    return false;
  }
  /*
  forward state to tailstamp and return true, 
  or buffer.front()->header.stamp if tailstamp is later than buffer.front()->header.stamp and return false.
  */
  if (buffer.empty()) {
    lock.unlock();
    if (wait_imu(10) < 0) return false;
    lock.lock();
  }
  bool ret = stamp_to_sec(buffer.front()->header.stamp) < tailstamp? false: true;
  stamp_t forward_tailstamp = std::min(tailstamp, stamp_to_sec(buffer.front()->header.stamp));
  set_state(forward_impl(get_state(), buffer.front(), forward_tailstamp));
  publish_tf(get_state());
  propagated_queue.push_back(get_state());
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
  static matrix18_t Fx = matrix18_t::Identity();
  Fx.block<3, 3>(0, 0) = so3_t::exp(-mean_ang_vel * dt).matrix();
  Fx.block<3, 3>(0, 9) = -matrix3_t::Identity() * dt;
  Fx.block<3, 3>(3, 6) = matrix3_t::Identity() * dt;
  Fx.block<3, 3>(6, 0) = -state.get_rotation().matrix() * mean_accel_skew * dt;
  Fx.block<3, 3>(6, 12) = -state.get_rotation().matrix() * dt;
  Fx.block<3, 3>(6, 15) = matrix3_t::Identity() * dt;

  static matrix18_t W = matrix18_t::Zero();
  W.block<3, 3>(0, 0).diagonal() = measure_noise.segment<3>(0) * dt * dt;
  W.block<3, 3>(6, 6) = state.get_rotation().matrix() * measure_noise.segment<3>(3).asDiagonal() * state.get_rotation().matrix().transpose() * dt * dt;
  W.block<3, 3>(9, 9).diagonal() = measure_noise.segment<3>(6) * dt * dt;
  W.block<3, 3>(12, 12).diagonal() = measure_noise.segment<3>(9) * dt * dt;

  static matrix18_t tail_covariance;
  tail_covariance = Fx * state.get_covariance() * Fx.transpose() + W;
  so3_t dR = so3_t::exp(mean_ang_vel * dt);
  vector3_t accel_world = state.get_rotation().matrix() * mean_accel + state.get_gravity();

  so3_t tail_rotation = state.get_rotation() * dR;
  vector3_t tail_translation = state.get_translation() + state.get_linear_velocity() * dt + 0.5 * accel_world * dt * dt;
  vector3_t tail_linear_velocity = state.get_linear_velocity() + accel_world * dt;
  state_t state_forwarded = state_t(tailstamp, 
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

} // namespace gs_lio