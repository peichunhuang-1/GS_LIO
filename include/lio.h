#ifndef GS_LIO_H
#define GS_LIO_H

#include "lidar_plugin.h"
#include "voxel.h"
#include "imu.h"

#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <geometry_msgs/msg/transform_stamped.hpp>

namespace gs_lio
{

class Residual
{
  public:
    scalar_t distance_to_plane = 0;
    scalar_t residual = 0;
    Eigen::RowVector<scalar_t, 6> H_row;
};

class Lio: public Imu
{
  enum status
  {
    IDLE,
    READY,
    INITIALED,
  };
  public:
    Lio(const std::string &name);
    ~Lio();
    void optimize() override;
    void reset(const state_t &state) override;
    stamp_t wait_lidar(int timeout_ms = 200);
    bool is_init() {return current_status == INITIALED;}
  private:
    std::atomic<status> current_status = IDLE;
    matrix18_t G = matrix18_t::Identity();
    matrix18_t H_T_H = matrix18_t::Identity();
    matrix18_t I_STATE = matrix18_t::Identity();

    // project lidar frame to imu frame
    vector3_t lidar_imu_extrinsic_translation = vector3_t::Zero();
    quaternion_t lidar_imu_extrinsic_orientation = quaternion_t::Identity();

    int lio_max_iter = 5;
    int radius_sigma_num = 5; // valid point to plane projection radius limit
    int distance_sigma_num = 3; // valid point to plane distance limit
    int valid_plane_threshold = 1000; // valid plane for initializing

    std::string imu_link;
    std::string lidar_link;

    bool try_initialize();

    pcl::PointCloud<pcl::PointXYZITC> undistorted_pointcloud(const state_t& current_state, const std::shared_ptr<pcl::PointCloud<pcl::PointXYZIT>> &raw_pointcloud);
    pcl::PointCloud<pcl::PointXYZITC> transform_pointcloud_to_world_frame(const pcl::PointCloud<pcl::PointXYZITC> &pointcloud, const state_t &state);
    
    std::shared_ptr<Residual> build_residual(const state_t& current_state, const pcl::PointXYZITC &pw, const pcl::PointXYZITC &pl, const std::shared_ptr<PlaneImpl> &plane);
    vector18_t ieskf(const std::vector<std::shared_ptr<Residual>> &residuals, const vector18_t &error_state);

    std::thread build_map_thread;
    std::shared_ptr<VoxelOctoTree> voxel_tree;
    std::shared_ptr<Lidar> lidar;

    std::shared_ptr<tf2_ros::Buffer> tf_buffer;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener;
};

} // namespace gs_lio

#endif