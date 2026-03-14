#include "plane.h"

namespace gs_lio
{
scalar_t Plane::PLANE_THRESHOLD = 0.0025;
int Plane::CONSTRUCT_THRESHOLD = 12;
// Assignment operator
Plane& Plane::operator=(const Plane& other)
{
  if (this != &other) {
    normal_ = other.normal_;
    main_axis_ = other.main_axis_;
    secondary_axis_ = other.secondary_axis_;
    center_ = other.center_;
    covariance_ = other.covariance_;
    uncertainty_ = other.uncertainty_;
    d_ = other.d_;
    radius_ = other.radius_;
    is_valid_ = other.is_valid_;
    mtx = std::make_shared<std::shared_mutex>();
  }
  return *this;
}

void Plane::insert_point(const pcl::PointXYZITC &point)
{
  std::unique_lock<std::shared_mutex> lock(*mtx);
  vector3_t pw_eigen = point.xyz().cast<scalar_t>();
  covariance_ = (point_num_ * (covariance_ + center_ * center_.transpose()) + pw_eigen * pw_eigen.transpose()) / (point_num_ + 1);
  // update center and covariance
  center_ = (point_num_ * center_ + pw_eigen) / (point_num_ + 1);
  covariance_ -= center_ * center_.transpose();
  p_covariance_ += point.covariance().cast<scalar_t>();
  px_covariance_ += point.x * point.covariance().cast<scalar_t>();
  py_covariance_ += point.y * point.covariance().cast<scalar_t>();
  pz_covariance_ += point.z * point.covariance().cast<scalar_t>();
  pxx_covariance_ += point.x * point.x * point.covariance().cast<scalar_t>();
  pxy_covariance_ += point.x * point.y * point.covariance().cast<scalar_t>();
  pxz_covariance_ += point.x * point.z * point.covariance().cast<scalar_t>();
  pyy_covariance_ += point.y * point.y * point.covariance().cast<scalar_t>();
  pyz_covariance_ += point.y * point.z * point.covariance().cast<scalar_t>();
  pzz_covariance_ += point.z * point.z * point.covariance().cast<scalar_t>();
  point_num_++;
}

void Plane::update()
{
  std::unique_lock<std::shared_mutex> lock(*mtx);
  if (point_num_ < CONSTRUCT_THRESHOLD) 
  {
    is_valid_ = false;
    return;
  }
  scalar_t covariance_norm = covariance_.norm();
  if (covariance_norm < 1e-6f) 
  {
    is_valid_ = false;
    return;
  }
  Eigen::EigenSolver<matrix3_t> es(covariance_);
  matrix3c_t eigen_vecs = es.eigenvectors();
  vector3c_t eigen_vals = es.eigenvalues();
  vector3_t eigen_vals_real = eigen_vals.real();
  matrix3_t::Index eigen_vals_min, eigen_vals_max;
  // find min and max eigenvalue indices
  eigen_vals_real.minCoeff(&eigen_vals_min);
  eigen_vals_real.maxCoeff(&eigen_vals_max);
  if (eigen_vals_real(eigen_vals_min) > Plane::PLANE_THRESHOLD) 
  {
    is_valid_ = false;
    return;
  }
  is_valid_ = true;
  vector3_t eigen_vec_min = eigen_vecs.real().col(eigen_vals_min);
  vector3_t eigen_vec_mid = eigen_vecs.real().col(3 - eigen_vals_min - eigen_vals_max);
  vector3_t eigen_vec_max = eigen_vecs.real().col(eigen_vals_max);
  matrix3_t J_Q = 1.0 / static_cast<scalar_t>(point_num_) * matrix3_t::Identity();

  scalar_t n_x_l1_l2 = point_num_ * (eigen_vals_real(eigen_vals_min) - eigen_vals_real(3 - eigen_vals_min - eigen_vals_max));
  scalar_t n_x_l1_l3 = point_num_ * (eigen_vals_real(eigen_vals_min) - eigen_vals_real(eigen_vals_max));
  
  scalar_t v1xv2y_minus_v1yv2x = eigen_vec_min.x() * eigen_vec_mid.y() - eigen_vec_min.y() * eigen_vec_mid.x();
  scalar_t v1xv2z_minus_v1zv2x = eigen_vec_min.x() * eigen_vec_mid.z() - eigen_vec_min.z() * eigen_vec_mid.x();
  scalar_t v1yv2z_minus_v1zv2y = eigen_vec_min.y() * eigen_vec_mid.z() - eigen_vec_min.z() * eigen_vec_mid.y();
  scalar_t v1xv3y_minus_v1yv3x = eigen_vec_min.x() * eigen_vec_max.y() - eigen_vec_min.y() * eigen_vec_max.x();
  scalar_t v1xv3z_minus_v1zv3x = eigen_vec_min.x() * eigen_vec_max.z() - eigen_vec_min.z() * eigen_vec_max.x();
  scalar_t v1yv3z_minus_v1zv3y = eigen_vec_min.y() * eigen_vec_max.z() - eigen_vec_min.z() * eigen_vec_max.y();

  matrix3_t Mx; Mx << 0, 0, 0, 
                      0, -v1xv2y_minus_v1yv2x / n_x_l1_l2, -v1xv2z_minus_v1zv2x / n_x_l1_l2, 
                      0, -v1xv3y_minus_v1yv3x / n_x_l1_l3, -v1xv3z_minus_v1zv3x / n_x_l1_l3;
  matrix3_t My; My << 0, 0, 0,
                       v1xv2y_minus_v1yv2x / n_x_l1_l2, 0, -v1yv2z_minus_v1zv2y / n_x_l1_l2, 
                       v1xv3y_minus_v1yv3x / n_x_l1_l3, 0, -v1yv3z_minus_v1zv3y / n_x_l1_l3;
  matrix3_t Mz; Mz << 0, 0, 0,
                       v1xv2z_minus_v1zv2x / n_x_l1_l2, v1yv2z_minus_v1zv2y / n_x_l1_l2, 0, 
                       v1xv3z_minus_v1zv3x / n_x_l1_l3, v1yv3z_minus_v1zv3y / n_x_l1_l3, 0;
  matrix3_t C; C << 0, 0, 0,
                    -(v1xv2y_minus_v1yv2x * center_.y() + v1xv2z_minus_v1zv2x * center_.z()) / n_x_l1_l2, (v1xv2y_minus_v1yv2x * center_.x() - v1yv2z_minus_v1zv2y * center_.z()) / n_x_l1_l2, (v1xv2z_minus_v1zv2x * center_.x() + v1yv2z_minus_v1zv2y * center_.y()) / n_x_l1_l2,
                    -(v1xv3y_minus_v1yv3x * center_.y() + v1xv3z_minus_v1zv3x * center_.z()) / n_x_l1_l3, (v1xv3y_minus_v1yv3x * center_.x() - v1yv3z_minus_v1zv3y * center_.z()) / n_x_l1_l3, (v1xv3z_minus_v1zv3x * center_.x() + v1yv3z_minus_v1zv3y * center_.y()) / n_x_l1_l3;
  matrix3_t V; V.col(0) = eigen_vec_min; V.col(1) = eigen_vec_mid; V.col(2) = eigen_vec_max;
  // update uncertainty
  uncertainty_.block<3,3>(0, 0) = 
    V * Mx * pxx_covariance_ * Mx.transpose() * V.transpose() +
    V * Mx * pxy_covariance_ * My.transpose() * V.transpose() + 
    V * Mx * pxz_covariance_ * Mz.transpose() * V.transpose() +
    V * Mx * px_covariance_ * C.transpose() * V.transpose() + 
    V * My * pxy_covariance_ * Mx.transpose() * V.transpose() +
    V * My * pyy_covariance_ * My.transpose() * V.transpose() +
    V * My * pyz_covariance_ * Mz.transpose() * V.transpose() +
    V * My * py_covariance_ * C.transpose() * V.transpose() +
    V * Mz * pxz_covariance_ * Mx.transpose() * V.transpose() +
    V * Mz * pyz_covariance_ * My.transpose() * V.transpose() +
    V * Mz * pzz_covariance_ * Mz.transpose() * V.transpose() +
    V * Mz * pz_covariance_ * C.transpose() * V.transpose();
  uncertainty_.block<3,3>(0, 3) = 
    V * Mx * px_covariance_ * J_Q.transpose() +
    V * My * py_covariance_ * J_Q.transpose() +
    V * Mz * pz_covariance_ * J_Q.transpose() +
    V * C * p_covariance_ * J_Q.transpose();
  uncertainty_.block<3,3>(3, 0) = uncertainty_.block<3,3>(0, 3).transpose();
  uncertainty_.block<3,3>(3, 3) = J_Q * p_covariance_ * J_Q.transpose();

  // update other members
  normal_ = eigen_vec_min.normalized();
  main_axis_ = eigen_vec_max.normalized();
  secondary_axis_ = eigen_vec_mid.normalized();
  radius_ = sqrt(eigen_vals_real(eigen_vals_max));
  d_ = -eigen_vec_min.dot(center_);
}

}