

#include "fake.hpp"

namespace rm_serial_driver {
    FakeSerialDriver::FakeSerialDriver(const rclcpp::NodeOptions &options)
        : Node("fake_serial_driver", options) {
        RCLCPP_INFO(this->get_logger(), "Fake serial driver start");

        //TF broadcaster
        timestamp_offset_ = this->declare_parameter("timestamp_offset", 0.0);

        tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

        //create publisher
        task_pub_ = this->create_publisher<std_msgs::msg::String>("/task_mode", 10);

        latency_pub_ = this->create_publisher<std_msgs::msg::Float64>("/latency", 10);

        marker_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("/aiming_point", 10);

        aim_time_info_pub_ = this->create_publisher<auto_aim_interfaces::msg::TimeInfo>("/time_info/aim", 10);
        //      buff_time_info_pub_ =
        // this->create_publisher<buff_interfaces::msg::TimeInfo>("/time_info/buff", 10);
        record_controller_pub_ = this->create_publisher<std_msgs::msg::String>("/record_controller", 10);

        detector_param_client_ = std::make_shared<rclcpp::AsyncParametersClient>(this, "armor_detector");

        reset_tracker_client_ = this->create_client<std_srvs::srv::Trigger>("/tracker/reset");

        change_target_client_ = this->create_client<std_srvs::srv::Trigger>("/tracker/change");

        receive_thread_ = std::thread(&FakeSerialDriver::receiveData, this);

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
        aiming_point_.lifetime = rclcpp::Duration::from_seconds(0.5);

        aim_sub_.subscribe(this, "/tracker/target", rclcpp::SensorDataQoS().get_rmw_qos_profile());
        aim_time_info_sub_.subscribe(this, "/time_info/aim");


        //将数据回传mcu
        aim_sync_ = std::make_unique<AimSync>(aim_syncpolicy(500), aim_sub_, aim_time_info_sub_);
        aim_sync_->registerCallback(
            std::bind(&FakeSerialDriver::sendArmorData, this,
                      std::placeholders::_1, std::placeholders::_2));
    }


    void FakeSerialDriver::receiveData() {
        rclcpp::Rate loop_rate(25); // 示例：限制为 100 Hz
        while (rclcpp::ok()) {
            //模拟接受数据
            rm_serial_driver::ReceiverPacket packet;
            set_fake_receiver_packet_random(packet, rm_auto_aim::RED);

            std_msgs::msg::String task;
            task.data = "aim";
            task_pub_->publish(task);


            geometry_msgs::msg::TransformStamped t;
            timestamp_offset_ = this->get_parameter("timestamp_offset").as_double();
            t.header.stamp = this->now() - rclcpp::Duration::from_seconds(timestamp_offset_);
            t.header.frame_id = "odom";
            t.child_frame_id = "gimbal_link";
            tf2::Quaternion q;
            q.setRPY(packet.roll, packet.pitch, packet.yaw);
            t.transform.rotation = tf2::toMsg(q);
            tf_broadcaster_->sendTransform(t);
            // RCLCPP_INFO(this->get_logger(), "yaw=%f", packet.yaw);
            // RCLCPP_INFO(this->get_logger(), "pitch=%f", packet.pitch);
            // RCLCPP_INFO(this->get_logger(), "roll=%f", packet.roll);

            auto_aim_interfaces::msg::TimeInfo aim_time_info;
            aim_time_info.header = t.header;
            aim_time_info_pub_->publish(aim_time_info);

            aiming_point_.header.stamp = this->now();
            aiming_point_.pose.position.x = packet.aim_x;
            aiming_point_.pose.position.y = packet.aim_y;
            aiming_point_.pose.position.z = packet.aim_z;
            marker_pub_->publish(aiming_point_);

            loop_rate.sleep();
        }
    }


    // ----------------------------------------------------------------------

    void FakeSerialDriver::set_fake_receiver_packet_random(
        rm_serial_driver::ReceiverPacket &packet,
        uint8_t target_color) {
        // --- 1. 分布定义（定义随机数的合理范围）---

        // --- 2. 赋值 ---

        // 包头和颜色
        packet.header = 0x5A;
        packet.detect_color = (target_color > 0) ? 1 : 0;
        packet.task_mode = 0;
        packet.reserved = 0;

        // 云台姿态 (Roll 设为 0.0，保持稳定)
        packet.roll = 0.0f;
        packet.pitch = 0;
        packet.yaw = 0;

        // 目标瞄准坐标
        packet.aim_x = 2;
        packet.aim_y = 0;
        packet.aim_z = 0;

        // 校验和
        packet.checksum = 0;
    }

    void FakeSerialDriver::sendArmorData(const auto_aim_interfaces::msg::Target::ConstSharedPtr msg,
                                         const auto_aim_interfaces::msg::TimeInfo::ConstSharedPtr time_info) {
        const static std::map<std::string, uint8_t> id_unit8_map{
            {"", 0}, {"outpost", 0}, {"1", 1}, {"2", 2},
            {"3", 3}, {"4", 4}, {"5", 5}, {"guard", 6}, {"base", 7}
        };
        try {
            rm_serial_driver::SendPacket packet;
            packet.state = msg->tracking ? 1 : 0;
            packet.id = id_unit8_map.at(msg->id);
            packet.armors_num = msg->armors_num;
            packet.x = msg->position.x;
            packet.y = msg->position.y;
            packet.z = msg->position.z;
            packet.yaw = msg->yaw;
            packet.vx = msg->velocity.x;
            packet.vy = msg->velocity.y;
            packet.vz = msg->velocity.z;
            packet.v_yaw = msg->v_yaw;
            packet.r1 = msg->radius_1;
            packet.r2 = msg->radius_2;
            packet.dz = msg->dz;

            crc16::Append_CRC16_Check_Sum(reinterpret_cast<uint8_t *>(&packet), sizeof(packet));

            std::vector<uint8_t> data = toVector(packet);

            // serial_driver_->port()->send(data);

            std_msgs::msg::Float64 latency;
            latency.data = (this->now() - msg->header.stamp).seconds() * 1000.0;

            //RCLCPP_DEBUG_STREAM(get_logger(), "Total latency: " + std::to_string(latency.data) + "ms");

            //RCLCPP_INFO(get_logger(), "\n\nDEBUG Total latency: %f ms \n\n", latency.data);

            latency_pub_->publish(latency);
        } catch (const std::exception &ex) {
            RCLCPP_ERROR(get_logger(), "Error while sending data: %s", ex.what());
            // reopenPort();
        }
    }
}


#include "rclcpp_components/register_node_macro.hpp"

RCLCPP_COMPONENTS_REGISTER_NODE(rm_serial_driver::FakeSerialDriver)

