#ifndef GS_LIO_PLANE_H
#define GS_LIO_PLANE_H

#include "types.h"
#include <climits>

namespace gs_lio
{

class Plane
{
public:
  virtual inline bool is_valid() const { return false; }
  virtual inline vector3_t normal() const { return vector3_t::Zero(); }
  virtual inline vector3_t main_axis() const { return vector3_t::Zero(); }
  virtual inline vector3_t secondary_axis() const { return vector3_t::Zero(); }
  virtual inline vector3_t center() const { return vector3_t::Zero(); }
  virtual inline scalar_t d() const { return 0; }
  virtual inline scalar_t radius() const { return 0; }
  virtual inline matrix3_t covariance() const { return matrix3_t::Zero(); }
  virtual inline matrix6_t uncertainty() const { return matrix6_t::Zero(6, 6); }
  virtual inline int point_num() const { return INT_MAX; }
  virtual void insert_point(const pcl::PointXYZITC &point) {}
  virtual void update() {}
};

class PlaneImpl : public Plane
{
public:
  static scalar_t PLANE_THRESHOLD;
  static int CONSTRUCT_THRESHOLD;
  // Default constructor
  PlaneImpl() : is_valid_(false), mtx(std::make_shared<std::mutex>()) {}
  
  // Copy constructor
  PlaneImpl(const PlaneImpl& other)
    : normal_(other.normal_),
      main_axis_(other.main_axis_),
      secondary_axis_(other.secondary_axis_),
      center_(other.center_),
      covariance_(other.covariance_),
      uncertainty_(other.uncertainty_),
      d_(other.d_),
      radius_(other.radius_),
      is_valid_(other.is_valid_),
      mtx(std::make_shared<std::mutex>()) {}
  // Assignment operator
  PlaneImpl& operator=(const PlaneImpl& other);
  
  virtual void insert_point(const pcl::PointXYZITC &point) override;
  virtual void update() override;

  virtual inline bool is_valid() const override { return is_valid_; }
  virtual inline vector3_t normal() const override { return normal_; }
  virtual inline vector3_t main_axis() const override { return main_axis_; }
  virtual inline vector3_t secondary_axis() const override { return secondary_axis_; }
  virtual inline vector3_t center() const override { return center_; }
  virtual inline scalar_t d() const override { return d_; }
  virtual inline scalar_t radius() const override { return radius_; }
  virtual inline matrix3_t covariance() const override { return covariance_; }
  virtual inline matrix6_t uncertainty() const override { return uncertainty_; }
  virtual inline int point_num() const override { return point_num_; }
private:
  std::shared_ptr<std::mutex> mtx;
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
  matrix6_t uncertainty_ = matrix6_t::Identity();

};

} // namespace gs_lio

#endif