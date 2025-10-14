// Copyright (C) 2022 ChenJun
// Copyright (C) 2024 Zheng Yu
// Licensed under the MIT License.

#ifndef ARMOR_DETECTOR__DETECTOR_NODE_HPP_
#define ARMOR_DETECTOR__DETECTOR_NODE_HPP_

// ROS
#include <geometry_msgs/msg/point.hpp>
#include <image_transport/image_transport.hpp>
#include <image_transport/publisher.hpp>
#include <image_transport/subscriber_filter.hpp>
#include <rclcpp/publisher.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/string.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

// STD
#include <memory>
#include <string>
#include <vector>
#include <oneapi/tbb/partitioner.h>
#include <openvino/core/node.hpp>

#include "detector.hpp"
#include "number_classifier.hpp"
#include "pnp_solver.hpp"
#include "auto_aim_interfaces/msg/armors.hpp"
#include "auto_aim_interfaces/msg/debug_armors.hpp"
#include <visualization_msgs/msg/marker_array.hpp>
#include "ament_index_cpp/get_package_share_directory.hpp"
#include "cv_bridge/cv_bridge.h"



namespace rm_auto_aim
{

class ArmorDetectorNode: public rclcpp::Node
{
public:
   explicit ArmorDetectorNode(const rclcpp::NodeOptions & options);
  ~ArmorDetectorNode();
  std::unique_ptr<Detector> detector_;
private:
  void imageCallback(const sensor_msgs::msg::Image::ConstSharedPtr img_msg);

    void processingLoop() ;
  void publishArmorsAndMarkers(
    const sensor_msgs::msg::Image::ConstSharedPtr &img_msg,
    const std::vector<Armor> &armors);


  // rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr result_pub_;

  cv::VideoCapture cap_;

  rclcpp::TimerBase::SharedPtr timer_;


  std::unique_ptr<Detector> initDetector();
  std::vector<Armor> detectArmors(const sensor_msgs::msg::Image::ConstSharedPtr & img_msg);



  void processFrameTestVideo();

  void createDebugPublishers();
  void destroyDebugPublishers();

  void publishMarkers();

  // //  task subscriber
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr task_sub_;
  bool is_aim_task_;
  void taskCallback(const std_msgs::msg::String::SharedPtr task_msg);

  // Armor Detector

  // // Detected armors publisher
  auto_aim_interfaces::msg::Armors armors_msg_;
  rclcpp::Publisher<auto_aim_interfaces::msg::Armors>::SharedPtr armors_pub_;

  // Visualization marker publisher
  visualization_msgs::msg::Marker armor_marker_;
  visualization_msgs::msg::Marker text_marker_;
  visualization_msgs::msg::MarkerArray marker_array_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;

  // Camera info part
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr cam_info_sub_;
  cv::Point2f cam_center_;
  std::shared_ptr<sensor_msgs::msg::CameraInfo> cam_info_;
  std::unique_ptr<PnPSolver> pnp_solver_;

  // Image subscrpition
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr img_sub_;

  // Debug information
  bool debug_;
  std::shared_ptr<rclcpp::ParameterEventHandler> debug_param_sub_;
  std::shared_ptr<rclcpp::ParameterCallbackHandle> debug_cb_handle_;
  // rclcpp::Publisher<auto_aim_interfaces::msg::DebugLights>::SharedPtr lights_data_pub_;
  rclcpp::Publisher<auto_aim_interfaces::msg::DebugArmors>::SharedPtr armors_data_pub_;
  // image_transport::Publisher binary_img_pub_;
  image_transport::Publisher number_img_pub_;
  image_transport::Publisher result_img_pub_;

    std::thread processing_thread_;
    // std::mutex mutex_;
    // sensor_msgs::msg::Image::ConstSharedPtr latest_img_;
    std::atomic<bool> running_ = true;



  std::mutex mutex_;
  sensor_msgs::msg::Image::ConstSharedPtr latest_img_;
  std::vector<Armor> latest_armors_;  // 存储检测结果
  bool new_detection_available_ = false;

};

}  // namespace rm_auto_aim

#endif  // ARMOR_DETECTOR__DETECTOR_NODE_HPP_
