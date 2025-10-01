//
// FakeSerialDriver.cpp
//
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/string.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include "auto_aim_interfaces/msg/target.hpp"
#include "auto_aim_interfaces/msg/time_info.hpp"

class FakeSerialDriver : public rclcpp::Node {
public:
    FakeSerialDriver() : Node("fake_serial_driver") {
        RCLCPP_INFO(this->get_logger(),"Fake serial driver start");

        timestamp_offset_ = this->declare_parameter("timestamp_offset", 0.0);

        tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

        // Pubs
        task_pub_ = this->create_publisher<std_msgs::msg::String>("/task_mode", 10);
        latency_pub_ = this->create_publisher<std_msgs::msg::Float64>("/latency", 10);
        marker_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("/aiming_point", 10);
        aim_time_info_pub_ = this->create_publisher<auto_aim_interfaces::msg::TimeInfo>("/time_info/aim", 10);
        record_controller_pub_ = this->create_publisher<std_msgs::msg::String>("/record_controller", 10);

        // Marker 初始化
        aiming_point_.header.frame_id = "odom";
        aiming_point_.ns = "aiming_point";
        aiming_point_.type = visualization_msgs::msg::Marker::SPHERE;
        aiming_point_.action = visualization_msgs::msg::Marker::ADD;
        aiming_point_.scale.x = aiming_point_.scale.y = aiming_point_.scale.z = 0.12;
        aiming_point_.color.r = 1.0;
        aiming_point_.color.g = 1.0;
        aiming_point_.color.b = 1.0;
        aiming_point_.color.a = 1.0;
        aiming_point_.lifetime = rclcpp::Duration::from_seconds(0.1);

        // 定时器模拟 MCU / 任务命令
        timer_task_ = this->create_wall_timer(
            std::chrono::seconds(2),
            std::bind(&FakeSerialDriver::publishFakeTask, this));

        // 定时器生成 fake target 数据
        timer_target_ = this->create_wall_timer(
            std::chrono::milliseconds(100),
            std::bind(&FakeSerialDriver::publishFakeTarget, this));
    }

private:
    void publishFakeTarget() {
    auto now = this->now();

    // --- 发布 fake target ---
    auto target = auto_aim_interfaces::msg::Target();
    target.header.stamp = now;
    target.position.x = 1.0;
    target.position.y = 0.5;
    target.position.z = 0.2;
    target.tracking = true;
    target.id = "1";
    target.armors_num = 1;
    target.yaw = 0.0;
    target.v_yaw = 0.0;
    target.velocity.x = 0.0;
    target.velocity.y = 0.0;
    target.velocity.z = 0.0;
    target.radius_1 = 0.1;
    target.radius_2 = 0.1;
    target.dz = 0.0;

    // --- 发布 TimeInfo ---
    auto time_info = auto_aim_interfaces::msg::TimeInfo();
    time_info.header.stamp = now;
    time_info.time = 0;
    aim_time_info_pub_->publish(time_info);

    // --- 发布 Marker ---
    aiming_point_.header.stamp = now;
    aiming_point_.pose.position.x = target.position.x;
    aiming_point_.pose.position.y = target.position.y;
    aiming_point_.pose.position.z = target.position.z;
    marker_pub_->publish(aiming_point_);

    // --- 发布 Latency ---
    std_msgs::msg::Float64 latency_msg;
    latency_msg.data = (this->now() - time_info.header.stamp).seconds();
    latency_pub_->publish(latency_msg);

    // --- 发布完整 TF 链 ---
    geometry_msgs::msg::TransformStamped t;

    // 1. odom -> gimbal_link
    t.header.stamp = now;
    t.header.frame_id = "odom";
    t.child_frame_id = "gimbal_link";
    t.transform.translation.x = 0.0;
    t.transform.translation.y = 0.0;
    t.transform.translation.z = 0.0;
    tf2::Quaternion q1;
    q1.setRPY(0, 0, 0);  // floating joint 可设为0
    t.transform.rotation = tf2::toMsg(q1);
    tf_broadcaster_->sendTransform(t);

    // 2. gimbal_link -> camera_link
    t.header.frame_id = "gimbal_link";
    t.child_frame_id = "camera_link";
    t.transform.translation.x = 0.0;
    t.transform.translation.y = 0.0;
    t.transform.translation.z = 0.0;
    tf2::Quaternion q2;
    q2.setRPY(0, 0, 0);  // 对应 xacro arg xyz/rpy，可修改
    t.transform.rotation = tf2::toMsg(q2);
    tf_broadcaster_->sendTransform(t);

    // 3. camera_link -> camera_optical_frame
    t.header.frame_id = "camera_link";
    t.child_frame_id = "camera_optical_frame";
    t.transform.translation.x = 0.0;
    t.transform.translation.y = 0.0;
    t.transform.translation.z = 0.0;
    tf2::Quaternion q3;
    q3.setRPY(-M_PI/2, 0, -M_PI/2);  // 对齐 URDF
    t.transform.rotation = tf2::toMsg(q3);
    tf_broadcaster_->sendTransform(t);
}



    void publishFakeTask() {
        std_msgs::msg::String msg;
        msg.data = "aim";
        task_pub_->publish(msg);
    }

    double timestamp_offset_;
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

    visualization_msgs::msg::Marker aiming_point_;

    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr task_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr latency_pub_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_;
    rclcpp::Publisher<auto_aim_interfaces::msg::TimeInfo>::SharedPtr aim_time_info_pub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr record_controller_pub_;

    rclcpp::TimerBase::SharedPtr timer_task_;
    rclcpp::TimerBase::SharedPtr timer_target_;
};

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<FakeSerialDriver>());
    rclcpp::shutdown();
    return 0;
}
