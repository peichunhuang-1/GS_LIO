#ifndef GS_LIO_VOXEL_H
#define GS_LIO_VOXEL_H

#include "types.h"
#include <omp.h>

#include <visualization_msgs/msg/marker_array.hpp>
#include <geometry_msgs/msg/point.hpp>
#include "plane.h"

#include "LRU.h"

#define VOXELMAP_HASH_P 116101
#define VOXELMAP_MAX_N 10000000000

namespace gs_lio
{

class VOXEL_LOCATION
{
public:
  int64_t x, y, z;

  VOXEL_LOCATION(int64_t vx = 0, int64_t vy = 0, int64_t vz = 0) : x(vx), y(vy), z(vz) {}

  bool operator==(const VOXEL_LOCATION &other) const { return (x == other.x && y == other.y && z == other.z); }
};

class Voxel: public std::enable_shared_from_this<Voxel>
{
  private:
    std::atomic<bool> need_update = false;
    bool fixed = false;
    std::shared_ptr<std::shared_mutex> operation_mtx;
  public:
    static constexpr size_t NUM_LEAVES = 8;  // octree structure
    static int MAX_POINT_NUM;
    static int MAX_LAYER;
    static scalar_t BASIC_VOXEL_SIZE;
    Voxel(const vector3_t &c, const scalar_t &ql, const int &l);
    ~Voxel();
    int layer = 0;
    vector3_t voxel_center = vector3_t::Zero();
    scalar_t quater_length = 0;
    std::shared_ptr<Plane> plane = nullptr;
    std::array<std::shared_ptr<Voxel>, NUM_LEAVES> leaves{nullptr, nullptr, nullptr, nullptr,
                                                          nullptr, nullptr, nullptr, nullptr};
    void UpdateVoxel();
    std::shared_ptr<Voxel> InsertPoint(const pcl::PointXYZITC &point_world);
    std::shared_ptr<PlaneImpl> GetPlaneIter(const pcl::PointXYZITC &point);
};

class VoxelOctoTree
{
  public:
    static int LRU_MAX_VOXEL_NUM;
    static std::atomic<int> VALID_PLANE_NUM;
    VoxelOctoTree(rclcpp::Node &node);
    ~VoxelOctoTree();
    void UpdateVoxelOctoTree(const pcl::PointCloud<pcl::PointXYZITC> &points_world);
    std::shared_ptr<PlaneImpl> GetPlane(const pcl::PointXYZITC &point_world, bool get_near_voxel = true);
  private:
    std::shared_ptr<std::shared_mutex> operation_mtx;
    std::vector<std::shared_ptr<Voxel>> update_voxels_vector;
    VOXEL_LOCATION get_hash(const pcl::PointXYZITC &point_world);
    VOXEL_LOCATION get_near(const VOXEL_LOCATION &basic_voxel_location, const pcl::PointXYZITC &point_world);
    
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub;
    visualization_msgs::msg::Marker marker;
    void initialize_marker();
    void publish_voxel_markers();
    rclcpp::Clock clock_;
  protected:
    std::shared_ptr<LRUCache<VOXEL_LOCATION, Voxel>> grids;
};

}

namespace std
{
using namespace gs_lio;
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

#endif