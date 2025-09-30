//
// Created by rm_autoaim on 2025/10/1.
//

#ifndef RM_SERIAL_DRIVER__RM_SERIAL_DRIVER_HPP_
#define RM_SERIAL_DRIVER__RM_SERIAL_DRIVER_HPP_

#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <tf2_ros/transform_broadcaster.h>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <rclcpp/publisher.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/subscription.hpp>
#include <serial_driver/serial_driver.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <visualization_msgs/msg/marker.hpp>

// C++ system
#include <future>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include "auto_aim_interfaces/msg/target.hpp"
#include "auto_aim_interfaces/msg/time_info.hpp"


// #include "buff_interfaces/msg/rune.hpp"
// #include "buff_interfaces/msg/time_info.hpp"

#endif //RM_SERIAL_DRIVER__RM_SERIAL_DRIVER_HPP_