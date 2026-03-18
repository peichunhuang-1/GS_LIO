#ifndef GS_LIO_LIVOX_PLUGIN_H
#define GS_LIO_LIVOX_PLUGIN_H

#include "lidar_plugin.h"
#include "livox_ros_driver2/msg/custom_msg.hpp"

namespace gs_lio
{

class Livox: public Lidar {
  public:
    Livox(rclcpp::Node &node);
  private: // in case different lidar has different process method.
    virtual void process_livox_msg(const livox_ros_driver2::msg::CustomMsg::SharedPtr msg);
    rclcpp::Subscription<livox_ros_driver2::msg::CustomMsg>::SharedPtr subscription;
    void cb(const livox_ros_driver2::msg::CustomMsg::SharedPtr msg);
}; 

}

#endif