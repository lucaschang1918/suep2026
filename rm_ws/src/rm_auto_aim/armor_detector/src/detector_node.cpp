#include "detector_node.hpp"

#include <ranges>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include "auto_aim_interfaces/msg/armors.hpp"


namespace rm_auto_aim {
    ArmorDetectorNode::ArmorDetectorNode(const rclcpp::NodeOptions &options)
        : Node("armor_detector", options), running_(false), initialized_(false) {

        RCLCPP_INFO(this->get_logger(), "Start DetectorNode");

        // ========== 第一步:初始化检测器(不涉及 ROS 通信) ==========
        detector_ = initDetector();
        RCLCPP_INFO(this->get_logger(), "Detector initialized");

        // ========== 第二步:创建发布器 ==========
        armors_pub_ = this->create_publisher<auto_aim_interfaces::msg::Armors>(
            "/detector/armors", rclcpp::SensorDataQoS());

        marker_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
            "/detector/marker", 10);

        // ========== 第三步:初始化 Marker ==========
        armor_marker_.ns = "armors";
        armor_marker_.action = visualization_msgs::msg::Marker::ADD;
        armor_marker_.type = visualization_msgs::msg::Marker::CUBE;
        armor_marker_.scale.x = 0.05;
        armor_marker_.scale.z = 0.125;
        armor_marker_.color.a = 1.0;
        armor_marker_.color.g = 0.5;
        armor_marker_.color.b = 1.0;
        armor_marker_.lifetime = rclcpp::Duration::from_seconds(0.1);

        text_marker_.ns = "classification";
        text_marker_.action = visualization_msgs::msg::Marker::ADD;
        text_marker_.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
        text_marker_.scale.z = 0.1;
        text_marker_.color.a = 1.0;
        text_marker_.color.r = 1.0;
        text_marker_.color.g = 1.0;
        text_marker_.color.b = 1.0;
        text_marker_.lifetime = rclcpp::Duration::from_seconds(0.1);

        // ========== 第四步:调试参数 ==========
        debug_ = this->declare_parameter("debug", true);
        if (debug_) {
            createDebugPublishers();
        }

        debug_param_sub_ = std::make_shared<rclcpp::ParameterEventHandler>(this);
        debug_cb_handle_ = debug_param_sub_->add_parameter_callback("debug", [this](const rclcpp::Parameter &p) {
            debug_ = p.as_bool();
            debug_ ? createDebugPublishers() : destroyDebugPublishers();
        });

        // ========== 第五步:任务模式订阅 ==========
        is_aim_task_ = true;
        task_sub_ = this->create_subscription<std_msgs::msg::String>(
            "/task_mode", 10, std::bind(&ArmorDetectorNode::taskCallback,
                                        this, std::placeholders::_1));

        // ========== 第六步:延迟初始化 PnP 和订阅器 ==========
        init_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100),
            [this]() {
                if (!initialized_) {
                    try {
                        this->initialize();
                        initialized_ = true;
                        init_timer_->cancel();
                        RCLCPP_INFO(this->get_logger(), "Initialization completed");
                    } catch (const std::exception& e) {
                        RCLCPP_ERROR(this->get_logger(), "Initialization failed: %s", e.what());
                        rclcpp::shutdown();
                    }
                }
            }
        );

        RCLCPP_INFO(this->get_logger(), "Constructor finished, waiting for initialization");
    }


    void ArmorDetectorNode::initialize() {
        RCLCPP_INFO(this->get_logger(), "Starting delayed initialization...");

        // ========== 初始化相机信息订阅(只订阅一次) ==========
        cam_info_sub_ = this->create_subscription<sensor_msgs::msg::CameraInfo>(
            "/camera_info", rclcpp::SensorDataQoS(),
            [this](sensor_msgs::msg::CameraInfo::ConstSharedPtr camera_info) {
                try {
                    cam_center_ = cv::Point2f(camera_info->k[2], camera_info->k[5]);
                    cam_info_ = std::make_shared<sensor_msgs::msg::CameraInfo>(*camera_info);
                    pnp_solver_ = std::make_unique<PnPSolver>(camera_info->k, camera_info->d);

                    RCLCPP_INFO(this->get_logger(), "Camera info received and PnP solver initialized");
                    cam_info_sub_.reset();  // 收到一次后立即取消订阅
                } catch (const std::exception& e) {
                    RCLCPP_ERROR(this->get_logger(), "Failed to initialize PnP solver: %s", e.what());
                }
            });

        // ========== 启动图像订阅 ==========
        img_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
            "/image_raw", rclcpp::SensorDataQoS(),
            std::bind(&ArmorDetectorNode::imageCallback, this, std::placeholders::_1));

        RCLCPP_INFO(this->get_logger(), "Image subscription started");

        // ========== 最后启动处理线程 ==========
        running_ = true;
        processing_thread_ = std::thread(&ArmorDetectorNode::processingLoop, this);

        RCLCPP_INFO(this->get_logger(), "Processing thread started");
    }


    ArmorDetectorNode::~ArmorDetectorNode() {
        RCLCPP_INFO(this->get_logger(), "Shutting down ArmorDetectorNode...");
        running_ = false;

        if (processing_thread_.joinable()) {
            processing_thread_.join();
        }

        RCLCPP_INFO(this->get_logger(), "ArmorDetectorNode destroyed");
    }


    void ArmorDetectorNode::processingLoop() {
        // 等待初始化完成
        while (rclcpp::ok() && running_ && !initialized_) {
            RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                                "Processing thread waiting for initialization...");
            std::this_thread::sleep_for(std::chrono::milliseconds(50));
        }

        if (!initialized_) {
            RCLCPP_ERROR(this->get_logger(), "Processing thread exiting - not initialized");
            return;
        }

        RCLCPP_INFO(this->get_logger(), "Processing loop started");

        while (rclcpp::ok() && running_) {
            sensor_msgs::msg::Image::ConstSharedPtr img_msg;
            {
                std::lock_guard<std::mutex> lock(mutex_);
                if (!latest_img_) {

                    continue;
                }
                img_msg = latest_img_;
            }

            try {


                auto armors = detectArmors(img_msg);

                if (is_aim_task_ && !armors.empty()) {
                    // 检查 pnp_solver_ 是否已初始化
                    if (!pnp_solver_) {
                        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                                            "PnP solver not ready, skipping frame");
                        continue;
                    }

                    armors_msg_.header = armor_marker_.header = text_marker_.header = img_msg->header;
                    armors_msg_.armors.clear();
                    marker_array_.markers.clear();
                    armor_marker_.id = 0;
                    text_marker_.id = 0;

                    auto_aim_interfaces::msg::Armor armor_msg;
                    for (const auto &armor : armors) {


                        cv::Mat rvec, tvec;
                        bool success = pnp_solver_->solvePnP(armor, rvec, tvec);

                        if (success) {

                            armor_msg.type = armor.classfication_result == "1" ? "large" : "small";
                            armor_msg.number = armor.number;

                            armor_msg.pose.position.x = tvec.at<double>(0);
                            armor_msg.pose.position.y = tvec.at<double>(1);
                            armor_msg.pose.position.z = tvec.at<double>(2);

                            cv::Mat rotation_matrix;
                            cv::Rodrigues(rvec, rotation_matrix);

                            if (rotation_matrix.empty() || rotation_matrix.rows != 3 || rotation_matrix.cols != 3) {
                                RCLCPP_WARN(this->get_logger(), "Invalid rotation matrix");
                                continue;
                            }

                            tf2::Matrix3x3 tf2_rotation_matrix(
                                rotation_matrix.at<double>(0, 0), rotation_matrix.at<double>(0, 1),
                                rotation_matrix.at<double>(0, 2), rotation_matrix.at<double>(1, 0),
                                rotation_matrix.at<double>(1, 1), rotation_matrix.at<double>(1, 2),
                                rotation_matrix.at<double>(2, 0), rotation_matrix.at<double>(2, 1),
                                rotation_matrix.at<double>(2, 2));

                            tf2::Quaternion tf2_q;
                            tf2_rotation_matrix.getRotation(tf2_q);
                            armor_msg.pose.orientation = tf2::toMsg(tf2_q);

                            armor_msg.distance_to_image_center = pnp_solver_->calculateDistanceToCenter(armor.center);

                            armor_msg.kpts.clear();
                            for (const auto &pt : {
                                     armor.objects_keypoints[1], armor.objects_keypoints[0],
                                     armor.objects_keypoints[3], armor.objects_keypoints[2]
                                 }) {
                                geometry_msgs::msg::Point point;
                                point.x = pt.x;
                                point.y = pt.y;
                                armor_msg.kpts.emplace_back(point);
                            }

                            // Fill the markers
                            armor_marker_.id++;
                            armor_marker_.scale.y = armor.classfication_result == "1" ? 0.23 : 0.135;
                            armor_marker_.pose = armor_msg.pose;
                            text_marker_.id++;
                            text_marker_.pose.position = armor_msg.pose.position;
                            text_marker_.pose.position.y -= 0.1;
                            text_marker_.text = armor.classfication_result;
                            armors_msg_.armors.emplace_back(armor_msg);
                            marker_array_.markers.emplace_back(armor_marker_);
                            marker_array_.markers.emplace_back(text_marker_);
                        } else {
                            RCLCPP_DEBUG(this->get_logger(), "PnP solve failed");
                        }
                    }


                        armors_pub_->publish(armors_msg_);
                        publishMarkers();

                }
            } catch (const cv::Exception& e) {
                RCLCPP_ERROR(this->get_logger(), "OpenCV exception: %s", e.what());
            } catch (const std::exception& e) {
                RCLCPP_ERROR(this->get_logger(), "Exception in processing loop: %s", e.what());
            }


        }

        RCLCPP_INFO(this->get_logger(), "Processing loop exiting normally");
    }


    void ArmorDetectorNode::taskCallback(const std_msgs::msg::String::SharedPtr task_msg) {
        std::string task_mode = task_msg->data;
        is_aim_task_ = task_mode == "aim" ? true : false;
    }


    void ArmorDetectorNode::imageCallback(const sensor_msgs::msg::Image::ConstSharedPtr img_msg) {
        std::lock_guard<std::mutex> lock(mutex_);
        latest_img_ = img_msg;
    }


    void ArmorDetectorNode::createDebugPublishers() {
        number_img_pub_ = image_transport::create_publisher(this, "/detector/number_img");
        result_img_pub_ = image_transport::create_publisher(this, "/detector/result_img");
    }


    void ArmorDetectorNode::destroyDebugPublishers() {
        if (armors_data_pub_) {
            armors_data_pub_.reset();
        }
        number_img_pub_.shutdown();
        result_img_pub_.shutdown();
    }


    void ArmorDetectorNode::publishMarkers() {
        using Marker = visualization_msgs::msg::Marker;
        armor_marker_.action = armors_msg_.armors.empty() ? Marker::DELETE : Marker::ADD;
        marker_pub_->publish(marker_array_);
    }


    std::unique_ptr<Detector> ArmorDetectorNode::initDetector() {
        rcl_interfaces::msg::ParameterDescriptor param_desc;
        param_desc.integer_range.resize(1);
        param_desc.integer_range[0].step = 1;
        param_desc.integer_range[0].from_value = 0;
        param_desc.integer_range[0].to_value = 255;
        int binary_thres = declare_parameter("binary_thres", 160, param_desc);

        param_desc.description = "0-RED, 1-BLUE";
        param_desc.integer_range[0].from_value = 0;
        param_desc.integer_range[0].to_value = 1;
        auto detect_color = declare_parameter("detect_color", RED, param_desc);

        std::string package_share_dir = ament_index_cpp::get_package_share_directory("armor_detector");
        std::string model_path_armor = package_share_dir + "/model/mobilenetv3_last_int_all_new/last.xml";

        float score_threshold = this->declare_parameter("score_threshold", 0.7);
        float nms_threshold = this->declare_parameter("nms_threshold", 0.3);

        auto detector = std::make_unique<Detector>(model_path_armor, score_threshold, nms_threshold, detect_color);

        std::string model_path_number = package_share_dir + "/model/number_classifier.onnx";
        auto label_path = package_share_dir + "/model/label.txt";
        auto threshold = this->declare_parameter("classifier_threshold", 0.7);
        std::vector<std::string> ignore_classes = {"negative"};
        detector->classifier =
                std::make_unique<NumberClassifier>(model_path_number, label_path, threshold, ignore_classes);

        return detector;
    }


    std::vector<Armor> ArmorDetectorNode::detectArmors(const sensor_msgs::msg::Image::ConstSharedPtr &img_msg) {
        // 检查图像有效性
        if (!img_msg || img_msg->data.empty()) {
            RCLCPP_WARN(this->get_logger(), "Invalid image message");
            return {};
        }

        auto img = cv_bridge::toCvShare(img_msg, "rgb8")->image;

        detector_->binary_thres = get_parameter("binary_thres").as_int();
        detector_->detect_color = get_parameter("detect_color").as_int();
        detector_->classifier->threshold = get_parameter("classifier_threshold").as_double();

        auto armors = detector_->detect(img);

        auto final_time = this->now();
        auto latency = (final_time - img_msg->header.stamp).seconds() * 1000;

        RCLCPP_DEBUG_STREAM(this->get_logger(), "Latency: " << latency << "ms");

        if (debug_) {
            if (!armors.empty()) {
                auto all_num_img = detector_->getAllNumbersImage();
                number_img_pub_.publish(
                    *cv_bridge::CvImage(img_msg->header, "mono8", all_num_img).toImageMsg());
            }

            cv::circle(img, cam_center_, 5, cv::Scalar(255, 0, 0), 2);

            std::stringstream latency_ss;
            latency_ss << "Latency: " << std::fixed << std::setprecision(2) << latency << "ms";
            auto latency_s = latency_ss.str();
            cv::putText(img, latency_s, cv::Point(10, 30),
                       cv::FONT_HERSHEY_SIMPLEX, 1.0, cv::Scalar(0, 255, 0), 2.0);
            result_img_pub_.publish(cv_bridge::CvImage(img_msg->header, "rgb8", img).toImageMsg());
        }

        return armors;
    }


    void ArmorDetectorNode::processFrameTestVideo() {
        cv::namedWindow("test result", cv::WINDOW_NORMAL);
        cv::resizeWindow("test result", 640, 640);
        cv::Mat frame;
        cap_ >> frame;
        if (frame.empty()) {
            RCLCPP_INFO(this->get_logger(), "Video end");
            rclcpp::shutdown();
            return;
        }

        auto armors = detector_->detect(frame);

        if (!armors.empty()) {
            auto all_num_img = detector_->getAllNumbersImage();
        }

        static auto last_time = std::chrono::steady_clock::now();
        auto now = std::chrono::steady_clock::now();
        double fps = 1000.0 / std::chrono::duration_cast<std::chrono::milliseconds>(now - last_time).count();
        last_time = now;

        cv::putText(frame, "FPS: " + std::to_string(int(fps)), cv::Point(10, 30),
                    cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(0, 255, 0), 2);
        cv::imshow("test result", frame);

        int k = cv::waitKey(1);
        if (k == 27) {
            rclcpp::shutdown();
            return;
        }
        else if (k == 'q') cv::waitKey(0);
    }

} // namespace rm_auto_aim


#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(rm_auto_aim::ArmorDetectorNode)