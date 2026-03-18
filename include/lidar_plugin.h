#ifndef GS_LIO_LIDAR_PLUGIN_H
#define GS_LIO_LIDAR_PLUGIN_H

#include "types.h"
#include <queue>

namespace gs_lio
{

class Lidar
{
  public:
    Lidar(rclcpp::Node &node);
    std::shared_ptr<pcl::PointCloud<pcl::PointXYZIT>> get_pointcloud();
    stamp_t wait_available(const int &timeout_ms); 
    matrix3_t point_covariance(const Eigen::Vector<scalar_t, 3> &point);
  protected:
    std::shared_ptr<std::shared_mutex> mtx;
    std::condition_variable_any cv;
    stamp_t tailstamp = -1;
    std::queue<stamp_t> tailstamps;

    int filter_ratio = 1; // filter points by ratio, e.g., filter_ratio = 2 means only keep 1 point for every 2 points
    scalar_t blind_zone = 0.1; // meter
    scalar_t range_covariance = 0.01; // meter^2
    scalar_t angle_covariance = 0.01; // rad^2

    std::queue<std::shared_ptr<pcl::PointCloud<pcl::PointXYZIT>>> raw_pointcloud_buffer;
};

}

#endif