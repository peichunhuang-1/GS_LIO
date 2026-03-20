cmake_minimum_required(VERSION 3.5)
project(gs-lio)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 23)
endif()

set(CMAKE_BUILD_TYPE "Release")

if(CMAKE_SYSTEM_PROCESSOR MATCHES "arm|aarch64|armv7|armv8")
  set(IS_ARM TRUE)
  message(STATUS "Detected ARM platform")
else()
  set(IS_ARM FALSE)
  message(STATUS "Detected x86/x64 platform")
endif()

if(CMAKE_BUILD_TYPE STREQUAL "Release")
  add_compile_options(-O3 -ffast-math)
  if(IS_ARM)
    add_compile_options(-mcpu=cortex-a72)
    message(STATUS "Using ARM optimization: -mcpu=cortex-a72")
  else()
    add_compile_options(-march=native -mtune=native)
    message(STATUS "Using x86 optimization: -march=native")
  endif()
  add_compile_definitions(
    EIGEN_NO_DEBUG
    EIGEN_DONT_ALIGN_STATICALLY
  )
endif()

find_package(mimalloc REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(PCL REQUIRED)
find_package(Boost REQUIRED COMPONENTS filesystem system serialization)
find_package(GTest REQUIRED)
find_package(livox_ros_driver2 REQUIRED)
FIND_PACKAGE(OpenCV REQUIRED core calib3d features2d highgui imgproc)
find_package(cv_bridge REQUIRED)
find_package(OpenMP QUIET)
find_package(tf2_ros REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(Sophus REQUIRED)

if(OpenMP_CXX_FOUND)
  message(STATUS "OpenMP found")
  add_compile_options(${OpenMP_CXX_FLAGS})
  add_link_options(${OpenMP_CXX_FLAGS})
endif()

include_directories(
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${EIGEN3_INCLUDE_DIR}
)

# add_subdirectory(thirdparty)

# Build state lib
add_library(gs_lio_state 
  src/state.cc
)

ament_target_dependencies(gs_lio_state
  rclcpp
  pcl_conversions
)

target_link_libraries(gs_lio_state 
  ${Boost_LIBRARIES} 
  mimalloc-static 
  Sophus::Sophus
  ${PCL_LIBRARIES}
)

# Build estimator lib
add_library(gs_lio_estimator 
  src/estimator.cc
)

ament_target_dependencies(gs_lio_estimator
  rclcpp
)

target_link_libraries(gs_lio_estimator gs_lio_state)

# Build imu lib
add_library(gs_lio_imu 
  src/imu.cc
)

ament_target_dependencies(gs_lio_imu
  rclcpp
  sensor_msgs
)

target_link_libraries(gs_lio_imu gs_lio_estimator)

# Build plane lib
add_library(gs_lio_plane
  src/plane.cc
)

target_link_libraries(gs_lio_plane
  ${PCL_LIBRARIES}
  ${EIGEN3_LIBRARIES}
  mimalloc-static 
  Sophus::Sophus
)

ament_target_dependencies(gs_lio_plane
  rclcpp
  pcl_conversions
)

# Build voxel lib
add_library(gs_lio_voxel
  src/voxel.cc
)

target_link_libraries(gs_lio_voxel
  gs_lio_plane
  ${PCL_LIBRARIES}
  ${EIGEN3_LIBRARIES}
  mimalloc-static 
)

ament_target_dependencies(gs_lio_voxel
  rclcpp
  pcl_conversions
  visualization_msgs
  geometry_msgs
)

# Build lidar plugin lib
add_library(gs_lio_lidar_plugin
  src/lidar_plugin.cc
)

target_link_libraries(gs_lio_lidar_plugin
  ${PCL_LIBRARIES}
  ${EIGEN3_LIBRARIES}
  Sophus::Sophus
  mimalloc-static 
)

ament_target_dependencies(gs_lio_lidar_plugin
  rclcpp
  pcl_conversions
)

# add plugins
add_subdirectory(plugins)

# Build lio lib
add_library(gs_lio 
  src/lio.cc
)

target_link_libraries(gs_lio 
  gs_lio_voxel
  gs_lio_lidar_plugin
  gs_lio_estimator
  plugins
)

ament_target_dependencies(gs_lio
  rclcpp
  pcl_conversions
  tf2_ros
  tf2_eigen
)

# Build lio node
add_executable(lio_node
  src/lio.cc
)

target_link_libraries(lio_node 
  gs_lio 
  gs_lio_imu
)

ament_target_dependencies(lio_node
  rclcpp
  pcl_conversions
  tf2_ros
  tf2_eigen
)

install(TARGETS
  lio_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
