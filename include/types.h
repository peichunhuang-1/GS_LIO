#ifndef GS_LIO_TYPES_H
#define GS_LIO_TYPES_H

#include <rclcpp/rclcpp.hpp>
#include <Eigen/Dense>
#include <sophus/se3.hpp>
#include <sophus/so3.hpp>

#define GRAVITY_CONSTANT 9.81

namespace gs_lio
{
using scalar_t = double;
using stamp_t = double;

#define pi_t static_cast<scalar_t>(std::acos(-1.0));

using vector_t = Eigen::Matrix<scalar_t, Eigen::Dynamic, 1>;
using matrix_t = Eigen::Matrix<scalar_t, Eigen::Dynamic, Eigen::Dynamic>;
using quaternion_t = Eigen::Quaternion<scalar_t>;

using vector2_t = Eigen::Matrix<scalar_t, 2, 1>;
using vector3_t = Eigen::Matrix<scalar_t, 3, 1>;
using vector4_t = Eigen::Matrix<scalar_t, 4, 1>;
using vector6_t = Eigen::Matrix<scalar_t, 6, 1>;

using vector2c_t = Eigen::Matrix<std::complex<scalar_t>, 2, 1>;
using vector3c_t = Eigen::Matrix<std::complex<scalar_t>, 3, 1>;

using matrix2_t = Eigen::Matrix<scalar_t, 2, 2>;
using matrix3_t = Eigen::Matrix<scalar_t, 3, 3>;
using matrix4_t = Eigen::Matrix<scalar_t, 4, 4>;
using matrix6_t = Eigen::Matrix<scalar_t, 6, 6>;

using matrix32_t = Eigen::Matrix<scalar_t, 3, 2>;

using matrix2c_t = Eigen::Matrix<std::complex<scalar_t>, 2, 2>;
using matrix3c_t = Eigen::Matrix<std::complex<scalar_t>, 3, 3>;

using se3_t = Sophus::SE3<scalar_t>;
using so3_t = Sophus::SO3<scalar_t>;

// time utilities
inline stamp_t stamp_to_sec(const builtin_interfaces::msg::Time& stamp) {return rclcpp::Time(stamp).seconds();}
inline rclcpp::Time sec_to_stamp(stamp_t timestamp) 
{
  int32_t sec = std::floor(timestamp);
  auto nanosec_d = (timestamp - std::floor(timestamp)) * 1e9;
  uint32_t nanosec = nanosec_d;
  return rclcpp::Time(sec, nanosec);
}
inline stamp_t now_sec() {return rclcpp::Clock().now().seconds();}
inline rclcpp::Time now_stamp() {return rclcpp::Clock().now();}
}

// point cloud types
#include <pcl_conversions/pcl_conversions.h>

namespace pcl
{
struct PointXYZIT
{
  PCL_ADD_POINT4D;                  // preferred way of adding a XYZ+padding
  float intensity;
  double timestamp;                 // seconds in absolute
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW   // make sure our new allocators are aligned
  inline Eigen::Map<Eigen::Vector3f> xyz()
  {
    return Eigen::Map<Eigen::Vector3f>(data);
  }
  inline Eigen::Map<const Eigen::Vector3f> xyz() const
  {
    return Eigen::Map<const Eigen::Vector3f>(data);
  }
} EIGEN_ALIGN16;                    // enforce SSE padding for correct memory alignment
struct EIGEN_ALIGN16 PointXYZITC
{
  PCL_ADD_POINT4D;
  float intensity;
  double timestamp;                 // seconds in absolute
  float covariance_data[9];         // row-major 3x3 covariance matrix
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  inline Eigen::Map<Eigen::Vector3f> xyz()
  {
    return Eigen::Map<Eigen::Vector3f>(data);
  }
  inline Eigen::Map<const Eigen::Vector3f> xyz() const
  {
    return Eigen::Map<const Eigen::Vector3f>(data);
  }
  inline Eigen::Map<Eigen::Matrix3f> covariance()
  {
    return Eigen::Map<Eigen::Matrix3f>(covariance_data);
  }
  inline Eigen::Map<const Eigen::Matrix3f> covariance() const
  {
    return Eigen::Map<const Eigen::Matrix3f>(covariance_data);
  }
} EIGEN_ALIGN16;
}

POINT_CLOUD_REGISTER_POINT_STRUCT (pcl::PointXYZIT,  
  (float, x, x)
  (float, y, y)
  (float, z, z)
  (float, intensity, intensity)
  (double, timestamp, timestamp)
)

POINT_CLOUD_REGISTER_POINT_STRUCT(pcl::PointXYZITC,
  (float, x, x)
  (float, y, y)
  (float, z, z)
  (float, intensity, intensity)
  (double, timestamp, timestamp)
  (float[9], covariance_data, covariance_data)
);

#endif