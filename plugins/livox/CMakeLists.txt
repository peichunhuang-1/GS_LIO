add_library(gs_lio_livox_plugin
  livox_plugin.h
  livox_plugin.cc
)

target_link_libraries(gs_lio_livox_plugin
  gs_lio_lidar_plugin
  mimalloc-static 
)

ament_target_dependencies(gs_lio_livox_plugin
  rclcpp
  sensor_msgs
  livox_ros_driver2
)