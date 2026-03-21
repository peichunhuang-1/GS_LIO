#ifndef GS_LIO_STATE_H
#define GS_LIO_STATE_H

#include "types.h"

namespace gs_lio
{
class state_t
{
public:
  state_t() = default;
  state_t(const stamp_t stamp, 
          const se3_t& transform, 
          const vector3_t& linear_velocity, 
          const vector3_t& angular_bias, 
          const vector3_t& linear_bias, 
          const vector3_t& gravity, 
          const matrix18_t& covariance)
    : timestamp(stamp), transform(transform), linear_velocity(linear_velocity), angular_bias(angular_bias), linear_bias(linear_bias), gravity(gravity), covariance(covariance) {}
  state_t(const stamp_t stamp, 
          const matrix3_t& rotation, 
          const vector3_t& translation, 
          const vector3_t& linear_velocity, 
          const vector3_t& angular_bias, 
          const vector3_t& linear_bias, 
          const vector3_t& gravity, 
          const matrix18_t& covariance)
    : timestamp(stamp), transform(rotation, translation), linear_velocity(linear_velocity), angular_bias(angular_bias), linear_bias(linear_bias), gravity(gravity), covariance(covariance) {}
  state_t(const stamp_t stamp, 
          const quaternion_t& rotation, 
          const vector3_t& translation, 
          const vector3_t& linear_velocity, 
          const vector3_t& angular_bias, 
          const vector3_t& linear_bias, 
          const vector3_t& gravity, 
          const matrix18_t& covariance)
    : timestamp(stamp), transform(rotation, translation), linear_velocity(linear_velocity), angular_bias(angular_bias), linear_bias(linear_bias), gravity(gravity), covariance(covariance) {}
  state_t(const stamp_t stamp,
          const so3_t& rotation,
          const vector3_t& translation,
          const vector3_t& linear_velocity,
          const vector3_t& angular_bias,
          const vector3_t& linear_bias,
          const vector3_t& gravity,
          const matrix18_t& covariance)
    : timestamp(stamp), transform(rotation, translation), linear_velocity(linear_velocity), angular_bias(angular_bias), linear_bias(linear_bias), gravity(gravity), covariance(covariance) {}
  state_t(const stamp_t stamp, 
          const vector3_t& rotation, 
          const vector3_t& translation, 
          const vector3_t& linear_velocity, 
          const vector3_t& angular_bias, 
          const vector3_t& linear_bias, 
          const vector3_t& gravity, 
          const matrix18_t& covariance)
    : timestamp(stamp), transform(so3_t::exp(rotation), translation), linear_velocity(linear_velocity), angular_bias(angular_bias), linear_bias(linear_bias), gravity(gravity), covariance(covariance) {}
  state_t(const state_t& other);
  ~state_t() = default;
  // operators
  state_t& operator=(const state_t& other);
  state_t operator+=(const vector18_t& delta);
  state_t operator-=(const vector18_t& delta);
  vector18_t operator-(const state_t& other) const;
  // getters
  stamp_t get_timestamp() const {return timestamp;}
  se3_t get_transform() const {return transform;}
  so3_t get_rotation() const {return transform.so3();}
  vector3_t get_translation() const {return transform.translation();}
  vector3_t get_linear_velocity() const {return linear_velocity;}
  vector3_t get_angular_bias() const {return angular_bias;}
  vector3_t get_linear_bias() const {return linear_bias;}
  vector3_t get_gravity() const {return gravity;}
  vector3_t get_imu_acceleration() const {return imu_acceleration;}
  vector3_t get_imu_angular_velocity() const {return imu_angular_velocity;}
  matrix18_t get_covariance() const {return covariance;}
  vector18_t to_vector() const
  {
    vector18_t state_vector;
    state_vector.segment<3>(0) = transform.so3().log();
    state_vector.segment<3>(3) = transform.translation();
    state_vector.segment<3>(6) = linear_velocity;
    state_vector.segment<3>(9) = angular_bias;
    state_vector.segment<3>(12) = linear_bias;
    state_vector.segment<3>(15) = gravity;
    return state_vector;
  }
  // setters
  void set_timestamp(const stamp_t stamp) {timestamp = stamp;}
  void set_transform(const se3_t& transform) {this->transform = transform;}
  void set_rotation(const so3_t& rotation) {this->transform.so3() = rotation;}
  void set_translation(const vector3_t& translation) {this->transform.translation() = translation;}
  void set_linear_velocity(const vector3_t& linear_velocity) {this->linear_velocity = linear_velocity;}
  void set_angular_bias(const vector3_t& angular_bias) {this->angular_bias = angular_bias;}
  void set_linear_bias(const vector3_t& linear_bias) {this->linear_bias = linear_bias;}
  void set_gravity(const vector3_t& gravity) {this->gravity = gravity;}
  void set_imu_acceleration(const vector3_t& imu_acceleration) {this->imu_acceleration = imu_acceleration;}
  void set_imu_angular_velocity(const vector3_t& imu_angular_velocity) {this->imu_angular_velocity = imu_angular_velocity;}
  void set_covariance(const matrix18_t& covariance) {this->covariance = covariance;}


  state_t slerp(const stamp_t &tailstamp) const;

  vector3_t measurement_project(const state_t& measured_state, 
                                const vector3_t &measurement, 
                                const matrix3_t &extrinsic_orientation,
                                const vector3_t &extrinsic_translation) const;
private:
  stamp_t timestamp;
  se3_t transform;
  vector3_t linear_velocity;
  vector3_t angular_bias;
  vector3_t linear_bias;
  vector3_t gravity = vector3_t(0.0, 0.0, -GRAVITY_CONSTANT);
  matrix18_t covariance = 0.01 * matrix18_t::Identity();
  // inertial measurements
  vector3_t imu_acceleration;
  vector3_t imu_angular_velocity;
};

}

#endif