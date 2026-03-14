#ifndef GS_LIO_VOXEL_H
#define GS_LIO_VOXEL_H

#include "types.h"
#include <omp.h>

#include <visualization_msgs/msg/marker_array.hpp>
#include <geometry_msgs/msg/point.hpp>

#include "LRU.h"

class VOXEL_LOCATION
{
public:
  int64_t x, y, z;

  VOXEL_LOCATION(int64_t vx = 0, int64_t vy = 0, int64_t vz = 0) : x(vx), y(vy), z(vz) {}

  bool operator==(const VOXEL_LOCATION &other) const { return (x == other.x && y == other.y && z == other.z); }
};

namespace std
{
template <> struct hash<VOXEL_LOCATION>
{
  int64_t operator()(const VOXEL_LOCATION &s) const
  {
    using std::hash;
    using std::size_t;
    return ((((s.z) * VOXELMAP_HASH_P) % VOXELMAP_MAX_N + (s.y)) * VOXELMAP_HASH_P) % VOXELMAP_MAX_N + (s.x);
  }
};
}

class Voxel
{
public:
  enum status
  {
    INIT,
    SUCCESS,
    FAILED,
    FIXED,
    ERROR
  }
private:
  status state = INIT;
  std::shared_ptr<std::shared_mutex> operation_mtx;
  std::atomic<int> point_num = 0;
  vector3_t voxel_center = vector3_t::Zero();
  std::shared_ptr<Plane> plane;

public:
  static int MAX_POINT_NUM;
  static int CONSTRUCT_THRESHOLD;
  static float PLANE_THRESHOLD;
  static float BASIC_VOXEL_SIZE;
  Voxel(const vector3_t &c);
  ~Voxel() = default;
  void InsertPoint(const pcl::PointXYZITC &point_world);
  std::shared_ptr<Plane> GetPlane(const pcl::PointXYZITC &point);
  void UpdateVoxel();
};

class VoxelMap
{
private:
  static int LRU_MAX_VOXEL_NUM;
  static std::atomic<int> valid_plane_num;
  std::shared_ptr<std::shared_mutex> operation_mtx;
  VOXEL_LOCATION get_hash(const pcl::PointXYZITC &point_world);
  VOXEL_LOCATION get_near(const VOXEL_LOCATION &basic_voxel_location, const pcl::PointXYZITC &point_world);
  std::shared_ptr<LRUCache<VOXEL_LOCATION, Voxel>> grids;
  std::vector<std::shared_ptr<Voxel>> update_voxels_vector;
public:
  VoxelMap();
  ~VoxelMap() = default;
  int GetValidPlaneNum() const { return valid_plane_num.load(); }
  void UpdateVoxelMap(const pcl::PointCloud<pcl::PointXYZITC> &points_world);
  std::shared_ptr<Plane> GetPlane(const pcl::PointXYZITC &point_world, bool get_near_voxel = true);
  std::shared_ptr<VOXEL_LOCATION> HasVoxel(const pcl::PointXYZITC &point_world);
};

#endif