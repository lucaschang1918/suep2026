//
// Created by rm_autoaim on 2025/10/4.
//

#ifndef RM_SERIAL_DRIVER_FAKE_HPP
#define RM_SERIAL_DRIVER_FAKE_HPP

#include <tf2_ros/transform_broadcaster.h>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/string.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include "auto_aim_interfaces/msg/target.hpp"
#include "auto_aim_interfaces/msg/time_info.hpp"
#include "auto_aim_interfaces/msg/armors.hpp"
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include "../include/rm_serial_driver/packet.hpp"
#include  "../include/rm_serial_driver/crc.hpp"
#include <random> // 核心随机数库
#include <chrono> // 用于生成更好的随机数种子
#include <serial_driver/serial_driver.hpp>

#include "../../rm_auto_aim/armor_detector/include/armor.hpp"

namespace rm_serial_driver {
    static std::default_random_engine generator(
        std::chrono::system_clock::now().time_since_epoch().count()
    );

    class FakeSerialDriver : public rclcpp::Node {
    public:
        explicit FakeSerialDriver(const rclcpp::NodeOptions &options);

        typedef message_filters::sync_policies::ApproximateTime<
           auto_aim_interfaces::msg::Target,
           auto_aim_interfaces::msg::TimeInfo> aim_syncpolicy;

        typedef message_filters::Synchronizer<aim_syncpolicy> AimSync;

    private:

        void getParams();

        void reopenPort();


        void receiveData();

        void set_fake_receiver_packet_random(
            rm_serial_driver::ReceiverPacket &packet,
            uint8_t target_color);

        void sendArmorData(const auto_aim_interfaces::msg::Target::ConstSharedPtr msg,
                           const auto_aim_interfaces::msg::TimeInfo::ConstSharedPtr time_info);


        //Serial port
        std::unique_ptr<IoContext> owned_ctx_;
        std::string device_name_;
        std::unique_ptr<drivers::serial_driver::SerialPortConfig> device_config_;
        std::unique_ptr<drivers::serial_driver::SerialDriver> serial_driver_;

        bool initial_set_param_ = false;
        uint8_t previous_receive_color_ = 0;


        double timestamp_offset_;
        std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

        visualization_msgs::msg::Marker aiming_point_;

        rclcpp::Publisher<std_msgs::msg::String>::SharedPtr task_pub_;
        rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr latency_pub_;
        rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_;
        rclcpp::Publisher<auto_aim_interfaces::msg::TimeInfo>::SharedPtr aim_time_info_pub_;
        rclcpp::Publisher<std_msgs::msg::String>::SharedPtr record_controller_pub_;
        rclcpp::Publisher<auto_aim_interfaces::msg::Armors>::SharedPtr armors_pub_;

        rclcpp::AsyncParametersClient::SharedPtr detector_param_client_;
        rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr reset_tracker_client_;
        rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr change_target_client_;

        message_filters::Subscriber<auto_aim_interfaces::msg::Target> aim_sub_;
        message_filters::Subscriber<auto_aim_interfaces::msg::TimeInfo> aim_time_info_sub_;


        std::thread receive_thread_;

        std::shared_ptr<AimSync> aim_sync_;

        rclcpp::TimerBase::SharedPtr timer_task_;
        rclcpp::TimerBase::SharedPtr timer_target_;
    };
}

#endif //RM_SERIAL_DRIVER_FAKE_HPP
