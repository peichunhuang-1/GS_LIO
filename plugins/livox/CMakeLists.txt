add_library(gs_lio_livox_plugin
  livox_plugin.h
  livox_plugin.cc
)

target_include_directories(gs_lio_livox_plugin 
PUBLIC
  ${CMAKE_CURRENT_SOURCE_DIR}
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

set(PLUGIN_TARGETS ${PLUGIN_TARGETS} gs_lio_livox_plugin PARENT_SCOPE)