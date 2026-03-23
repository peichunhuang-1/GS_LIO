#ifndef GS_LIO_LIDAR_PLUGIN_H
#define GS_LIO_LIDAR_PLUGIN_H

#include "types.h"
#include <queue>
#include <unordered_map>
#include <functional>

namespace gs_lio
{

class Lidar
{
  public:
    Lidar(rclcpp::Node &node);
    virtual std::shared_ptr<pcl::PointCloud<pcl::PointXYZIT>> get_pointcloud();
    virtual stamp_t wait_available(const int &timeout_ms); 
    virtual matrix3_t point_covariance(const Eigen::Vector<scalar_t, 3> &point);
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

class LidarPluginFactory 
{
  public:
    using Creator = std::function<std::unique_ptr<Lidar>(rclcpp::Node&)>;
    static LidarPluginFactory& instance();
    void register_creator(const std::string& name, Creator c);
    std::unique_ptr<Lidar> create(const std::string& name, rclcpp::Node& node);
  private:
    std::unordered_map<std::string, Creator> creators_;
};

} // namespace gs_lio

#define REGISTER_LIDAR(NAME, TYPE)                                     \
namespace gs_lio                                                       \
{                                                                      \
struct TYPE##Registrar {                                               \
    TYPE##Registrar() {                                                \
        LidarPluginFactory::instance().register_creator(               \
            NAME,                                                      \
            [](rclcpp::Node &node) -> std::unique_ptr<Lidar> {         \
                return std::make_unique<TYPE>(node);                   \
            }                                                          \
        );                                                             \
    }                                                                  \
};                                                                     \
static TYPE##Registrar global_##TYPE##_registrar;                      \
} // namespace gs_lio

#endif