#include "state.h"

namespace gs_lio
{
state_t::state_t(const state_t& other)
  : timestamp(other.timestamp), transform(other.transform), linear_velocity(other.linear_velocity), angular_bias(other.angular_bias), linear_bias(other.linear_bias), gravity(other.gravity), imu_acceleration(other.imu_acceleration), imu_angular_velocity(other.imu_angular_velocity), covariance(other.covariance) {}


state_t state_t::slerp(const stamp_t &tailstamp) const {
  double dt = tailstamp - timestamp;
  if (dt < 0) throw std::runtime_error("slerp time difference cannot be nagtive");
  vector3_t mean_accel = imu_acceleration * GRAVITY_CONSTANT - linear_bias;
  vector3_t mean_ang_vel = imu_angular_velocity - angular_bias;

  so3_t dR = so3_t::exp(mean_ang_vel * dt);
  vector3_t accel_world = get_rotation().matrix() * mean_accel + get_gravity();
  so3_t tail_rotation = get_rotation() * dR;
  vector3_t tail_translation = get_translation() + get_linear_velocity() * dt + 0.5 * accel_world * dt * dt;
  vector3_t tail_linear_velocity = get_linear_velocity() + accel_world * dt;
  auto state_forwarded = state_t(tailstamp, 
                                 tail_rotation, 
                                 tail_translation, 
                                 tail_linear_velocity, 
                                 get_angular_bias(), 
                                 get_linear_bias(), 
                                 get_gravity(), 
                                 matrix3_t::Identity());
  return state_forwarded;
}

vector3_t state_t::measurement_project(const state_t& measured_state, 
                                       const vector3_t &measurement, 
                                       const matrix3_t &extrinsic_orientation,
                                       const vector3_t &extrinsic_translation) const {
  vector3_t p = measured_state.get_translation();
  vector3_t measurement_world = measured_state.get_rotation() * (extrinsic_orientation * measurement + extrinsic_translation) + p;
  return extrinsic_orientation.transpose() * (get_rotation().matrix().transpose() * (measurement_world - get_translation()) - extrinsic_translation);
}

state_t& state_t::operator=(const state_t& other)
{
  if (this != &other)
  {
    timestamp = other.timestamp;
    transform = other.transform;
    linear_velocity = other.linear_velocity;
    angular_bias = other.angular_bias;
    linear_bias = other.linear_bias;
    gravity = other.gravity;
    imu_acceleration = other.imu_acceleration;
    imu_angular_velocity = other.imu_angular_velocity;
    covariance = other.covariance;
  }
  return *this;
}

state_t state_t::operator+=(const vector_t& delta) const
{
  state_t result(*this);
  result.transform.so3() = result.transform.so3() * so3_t::exp(delta.segment<3>(0).eval());
  result.transform.translation() += delta.segment<3>(3);
  result.linear_velocity += delta.segment<3>(6);
  result.angular_bias += delta.segment<3>(9);
  result.linear_bias += delta.segment<3>(12);
  result.gravity += delta.segment<3>(15);
  // covariance is not updated here, as it should be handled by the estimator
  return result;
}

state_t state_t::operator-=(const vector_t& delta) const
{
  state_t result(*this);
  result.transform.so3() = so3_t::exp(delta.segment<3>(0).eval()).inverse() * result.transform.so3();
  result.transform.translation() -= delta.segment<3>(3);
  result.linear_velocity -= delta.segment<3>(6);
  result.angular_bias -= delta.segment<3>(9);
  result.linear_bias -= delta.segment<3>(12);
  result.gravity -= delta.segment<3>(15);
  // covariance is not updated here, as it should be handled by the estimator
  return result;
}

vector_t state_t::operator-(const state_t& other) const
{
  vector_t delta(18);
  delta.segment<3>(0) = (other.transform.so3().inverse() * transform.so3()).log();
  delta.segment<3>(3) = transform.translation() - other.transform.translation();
  delta.segment<3>(6) = linear_velocity - other.linear_velocity;
  delta.segment<3>(9) = angular_bias - other.angular_bias;
  delta.segment<3>(12) = linear_bias - other.linear_bias;
  delta.segment<3>(15) = gravity - other.gravity;
  return delta;
}

}