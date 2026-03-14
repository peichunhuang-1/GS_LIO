#include "state.h"

namespace gs_lio
{
state_t::state_t(const state_t& other)
  : timestamp(other.timestamp), transform(other.transform), linear_velocity(other.linear_velocity), angular_bias(other.angular_bias), linear_bias(other.linear_bias), gravity(other.gravity), imu_acceleration(other.imu_acceleration), imu_angular_velocity(other.imu_angular_velocity), covariance(other.covariance) {}

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