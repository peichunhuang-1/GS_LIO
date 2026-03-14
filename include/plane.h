#ifndef GS_LIO_PLANE_H
#define GS_LIO_PLANE_H

#include "types.h"

namespace gs_lio
{

class Plane
{
public:
  static scalar_t PLANE_THRESHOLD;
  static int CONSTRUCT_THRESHOLD;
  // Default constructor
  Plane() : is_valid_(false), mtx(std::make_shared<std::shared_mutex>()) {}
  
  // Copy constructor
  Plane(const Plane& other)
    : normal_(other.normal_),
      main_axis_(other.main_axis_),
      secondary_axis_(other.secondary_axis_),
      center_(other.center_),
      covariance_(other.covariance_),
      uncertainty_(other.uncertainty_),
      d_(other.d_),
      radius_(other.radius_),
      is_valid_(other.is_valid_),
      mtx(std::make_shared<std::shared_mutex>()) {}
  // Assignment operator
  Plane& operator=(const Plane& other);
  
  void insert_point(const pcl::PointXYZITC &point);
  void update();

  bool is_valid() const { std::shared_lock<std::shared_mutex> lock(*mtx); return is_valid_; }
  vector3_t normal() const { std::shared_lock<std::shared_mutex> lock(*mtx); return normal_; }
  vector3_t main_axis() const { std::shared_lock<std::shared_mutex> lock(*mtx); return main_axis_; }
  vector3_t secondary_axis() const { std::shared_lock<std::shared_mutex> lock(*mtx); return secondary_axis_; }
  vector3_t center() const { std::shared_lock<std::shared_mutex> lock(*mtx); return center_; }
  scalar_t d() const { std::shared_lock<std::shared_mutex> lock(*mtx); return d_; }
  scalar_t radius() const { std::shared_lock<std::shared_mutex> lock(*mtx); return radius_; }
  matrix3_t covariance() const { std::shared_lock<std::shared_mutex> lock(*mtx); return covariance_; }
  matrix_t uncertainty() const { std::shared_lock<std::shared_mutex> lock(*mtx); return uncertainty_; }

private:
  std::shared_ptr<std::shared_mutex> mtx;
  bool is_valid_ = false;
  int point_num_ = 0;
  vector3_t normal_;
  vector3_t main_axis_;
  vector3_t secondary_axis_;
  vector3_t center_;
  scalar_t d_ = 0;
  scalar_t radius_ = 0;

  matrix3_t covariance_;
  matrix3_t p_covariance_ = matrix3_t::Zero();
  matrix3_t px_covariance_ = matrix3_t::Zero();
  matrix3_t py_covariance_ = matrix3_t::Zero();
  matrix3_t pz_covariance_ = matrix3_t::Zero();
  matrix3_t pxx_covariance_ = matrix3_t::Zero();
  matrix3_t pxy_covariance_ = matrix3_t::Zero();
  matrix3_t pxz_covariance_ = matrix3_t::Zero();
  matrix3_t pyy_covariance_ = matrix3_t::Zero();
  matrix3_t pyz_covariance_ = matrix3_t::Zero();
  matrix3_t pzz_covariance_ = matrix3_t::Zero();
  matrix_t uncertainty_ = matrix_t::Identity(6, 6);
};

}

#endif