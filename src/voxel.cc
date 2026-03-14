#include "voxel.h"

int Voxel::MAX_POINT_NUM = 50;
int Voxel::CONSTRUCT_THRESHOLD = 12;
float Voxel::PLANE_THRESHOLD = 0.0025;
float Voxel::BASIC_VOXEL_SIZE = 0.5;
int VoxelMap::LRU_MAX_VOXEL_NUM = 100000;
std::atomic<int> VoxelMap::valid_plane_num{0};

Voxel::Voxel(const vector3_t &c) : voxel_center(c)
{
  operation_mtx = std::make_shared<std::shared_mutex>();
  plane = std::make_shared<Plane>();
}

void Voxel::create_plane()
{
  scalar_t covariance_norm = covariance.norm();
  if (covariance_norm < 1e-6f || point_num.load() == 0) {
    state = INIT;
    return;
  }
  // PCA
  Eigen::EigenSolver<matrix3_t> es(plane->covariance);
  matrix3c_t eigen_vecs = es.eigenvectors();
  vector3c_t eigen_vals = es.eigenvalues();
  vector3_t eigen_vals_real = eigen_vals.real();
  matrix3_t::Index eigen_vals_min, eigen_vals_max;
  // find min and max eigenvalue indices
  eigen_vals_real.minCoeff(&eigen_vals_min);
  eigen_vals_real.maxCoeff(&eigen_vals_max);
  if (eigen_vals_real(eigen_vals_min) > PLANE_THRESHOLD)
  {
    plane->is_valid = false;
    state = FAILED;
    return;
  }
  vector3_t eigen_vec_min = eigen_vecs.real().col(eigen_vals_min);
  vector3_t eigen_vec_mid = eigen_vecs.real().col(3 - eigen_vals_min - eigen_vals_max);
  vector3_t eigen_vec_max = eigen_vecs.real().col(eigen_vals_max);
  
  
}