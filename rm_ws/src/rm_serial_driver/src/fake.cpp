

#include "fake.hpp"

namespace rm_serial_driver {
    FakeSerialDriver::FakeSerialDriver(const rclcpp::NodeOptions &options)
        : Node("fake_serial_driver", options), owned_ctx_{new IoContext(2)},
          serial_driver_{new drivers::serial_driver::SerialDriver(*owned_ctx_)} {
        RCLCPP_INFO(this->get_logger(), "Fake serial driver start");

        getParams();

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

        try {
            serial_driver_->init_port(device_name_, *device_config_);
            if (!serial_driver_->port()->is_open()) {
                serial_driver_->port()->open();
                receive_thread_ = std::thread(&FakeSerialDriver::receiveData, this);
            }
        } catch (const std::exception &ex) {
            RCLCPP_ERROR(
                get_logger(), "Error creating serial port : %s - %s", device_name_.c_str(), ex.what());
            throw ex;
        }

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

    void FakeSerialDriver::getParams() {
        using FlowControl = drivers::serial_driver::FlowControl;
        using Parity = drivers::serial_driver::Parity;
        using StopBits = drivers::serial_driver::StopBits;

        uint32_t baud_rate{};
        auto fc = FlowControl::NONE;
        auto pt = Parity::NONE;
        auto sb = StopBits::ONE;

        try {
            device_name_ = declare_parameter("device_name", std::string(""));
        } catch (rclcpp::ParameterTypeException &ex) {
            RCLCPP_ERROR(get_logger(), "The baud_rate provided was invalid");
            throw ex;
        }

        try {
            const auto fc_string = declare_parameter("flow_control", std::string(""));

            if (fc_string == "none") {
                fc = FlowControl::NONE;
            } else if (fc_string == "hardware") {
                fc = FlowControl::HARDWARE;
            } else if (fc_string == "software") {
                fc = FlowControl::SOFTWARE;
            } else {
                throw std::invalid_argument{
                    "The flow_control parameter must be one of: NONE, SOFTWARE, or "
                    "HARDWARE."
                };
            }
        } catch (rclcpp::ParameterTypeException &ex) {
            RCLCPP_ERROR(get_logger(), "The flow_control provided was invalid");
            throw ex;
        }

        try {
            const auto pt_string = declare_parameter<std::string>("parity", "");

            if (pt_string == "none") {
                pt = Parity::NONE;
            } else if (pt_string == "odd") {
                pt = Parity::ODD;
            } else if (pt_string == "even") {
                pt = Parity::EVEN;
            } else {
                throw std::invalid_argument{"The parity parameter must be one of: none, odd, or even."};
            }
        } catch (rclcpp::ParameterTypeException &ex) {
            RCLCPP_ERROR(get_logger(), "The parity provided was invalid");
            throw ex;
        }

        try {
            const auto sb_string = declare_parameter<std::string>("stop_bits", "");

            if (sb_string == "1" || sb_string == "1.0") {
                sb = StopBits::ONE;
            } else if (sb_string == "1.5") {
                sb = StopBits::ONE_POINT_FIVE;
            } else if (sb_string == "2" || sb_string == "2.0") {
                sb = StopBits::TWO;
            } else {
                throw std::invalid_argument{"The stop_bits parameter must be one of: 1, 1.5, or 2."};
            }
        } catch (rclcpp::ParameterTypeException &ex) {
            RCLCPP_ERROR(get_logger(), "The stop_bits provided was invalid");
            throw ex;
        }

        device_config_ =
                std::make_unique<drivers::serial_driver::SerialPortConfig>(baud_rate, fc, pt, sb);
    }

#if 1
    ///调试使用
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
#endif

//正常运行使用
#if 0
    void FakeSerialDriver::receiveData() {

        std::vector<uint8_t> header(1);
        std::vector<uint8_t> data;
        data.reserve(sizeof(ReceiverPacket));
        while (rclcpp::ok()) {

            try {
                serial_driver_->port()->receive(header);

                if (header[0] == 0x5A) {
                    data.resize(sizeof(ReceiverPacket) - 1);
                    serial_driver_->port()->receive(data);

                    data.insert(data.begin(), header[0]);
                    ReceiverPacket packet = fromVector(&data);

                    bool crc_ok = crc16::Verify_CRC16_Check_Sum(reinterpret_cast<const uint8_t *>(
                                                                    &packet), sizeof(packet));

                    if (crc_ok) {
                        if (!initial_set_param_ || packet.detect_color != previous_receive_color_) {
                            setParam(rclcpp::Parameter("detect_color", packet.detect_color));
                            previous_receive_color_ = packet.detect_color;
                        }
                        // RCLCPP_INFO(get_logger(), "CRC OK");

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
                        RCLCPP_INFO(this->get_logger(), "yaw=%f", packet.yaw);
                        RCLCPP_INFO(this->get_logger(), "pitch=%f", packet.pitch);
                        RCLCPP_INFO(this->get_logger(), "roll=%f", packet.roll);

                        auto_aim_interfaces::msg::TimeInfo aim_time_info;
                        aim_time_info.header = t.header;
                        aim_time_info_pub_->publish(aim_time_info);

                        aiming_point_.header.stamp = this->now();
                        aiming_point_.pose.position.x = packet.aim_x;
                        aiming_point_.pose.position.y = packet.aim_y;
                        aiming_point_.pose.position.z = packet.aim_z;
                        marker_pub_->publish(aiming_point_);


                    }
                }
            } catch (const std::exception &ex) {
                RCLCPP_ERROR_THROTTLE(
                    get_logger(), *get_clock(), 20,
                    "Error while rceiveing data: %s", ex.what());
                reopenPort();
            }
        }
    }
#endif


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
        // packet.reserved = 0;

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

    void FakeSerialDriver::reopenPort() {
        RCLCPP_WARN(get_logger(), "Attempting to reopen port");
        try {
            if (serial_driver_->port()->is_open()) {
                serial_driver_->port()->close();
            }
            serial_driver_->port()->open();
            RCLCPP_INFO(get_logger(), "Successfully reopened port");
        } catch (const std::exception &ex) {
            RCLCPP_ERROR(get_logger(), "Error while reopening port: %s", ex.what());
            if (rclcpp::ok()) {
                rclcpp::sleep_for(std::chrono::seconds(1));
                reopenPort();
            }
        }
    }

    void FakeSerialDriver::setParam(const rclcpp::Parameter &param) {
        if (!detector_param_client_->service_is_ready()) {
            RCLCPP_WARN(get_logger(), "Service not ready, skipping parameter set");
            return;
        }
        if (
            !set_param_future_.valid() ||
            set_param_future_.wait_for(std::chrono::seconds(0)) == std::future_status::ready) {
            RCLCPP_INFO(get_logger(), "Setting detect_color to %ld...", param.as_int());
            set_param_future_ = detector_param_client_->set_parameters(
                {param}, [this,param](const ResultFuturePtr &results) {
                    for (const auto &result: results.get()) {
                        if (!result.successful) {
                            RCLCPP_ERROR(get_logger(), "Failed to set parameter: %s", result.reason.c_str());
                            return;
                        }
                    }
                    RCLCPP_INFO(get_logger(), "Successfully set detect_color to %ld!", param.as_int());
                    initial_set_param_ = true;
                });
            }
    }

    void FakeSerialDriver::sendArmorData(const auto_aim_interfaces::msg::Target::ConstSharedPtr msg,
                                         const auto_aim_interfaces::msg::TimeInfo::ConstSharedPtr time_info) {
        const static std::map<std::string, uint8_t> id_unit8_map{
            {"", 0}, {"outpost", 0}, {"1", 1}, {"2", 2},
            {"3", 3}, {"4", 4}, {"5", 5}, {"guard", 6}, {"base", 7}
        };
        try {
            rm_serial_driver::SendPacket packet;
            packet.state = msg->tracking ? 1 : 0;       //ture / false
            packet.id = id_unit8_map.at(msg->id);       //6是哨兵 0是空
            packet.armors_num = msg->armors_num;        //装甲板的个数
            packet.x = msg->position.x;                 //预测x
            packet.y = msg->position.y;                 //y方向和ayw方向一样，在相机视角都是左正右负
            packet.z = msg->position.z;
            packet.yaw = msg->yaw * M_PI / 180;           //预测yaw，观测者左边正右边负  这个是机器人中心相对装甲板夹角
            packet.vx = msg->velocity.x;                //预测运动向量
            packet.vy = msg->velocity.y;
            packet.vz = msg->velocity.z;
            packet.v_yaw = msg->v_yaw;
            packet.r1 = msg->radius_1;
            packet.r2 = msg->radius_2;
            packet.dz = msg->dz;                    //修正高低装甲板的高度差
            //radius_1 和 radius_2 分别表示上下两层装甲板相对机器人中心的水平半径，
           // dz 表示它们在垂直方向的高度差。
           // 三者共同定义了装甲板在三维空间中相对于机器人中心的完整几何位置。

            crc16::Append_CRC16_Check_Sum(reinterpret_cast<uint8_t *>(&packet), sizeof(packet));

            std::vector<uint8_t> data = toVector(packet);

            serial_driver_->port()->send(data);

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

