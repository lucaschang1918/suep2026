cmake_minimum_required(VERSION 3.10)

project(armor_detector)

# C++20
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# ROS2 基础依赖
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(image_transport REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(rclcpp_components REQUIRED)

# OpenCV
find_package(OpenCV REQUIRED)

# OpenVINO
find_package(OpenVINO REQUIRED)
find_package(TBB REQUIRED)

# 其他包依赖
find_package(auto_aim_interfaces REQUIRED)

# 源文件
set(SOURCES
        src/detector_node.cpp
        src/detector.cpp
        src/number_classifier.cpp
        src/pnp_solver.cpp
)

# 可执行文件
add_executable(${PROJECT_NAME}_node ${SOURCES})

# 头文件路径（只保留本项目 include，依赖包交给 ament_target_dependencies）
target_include_directories(${PROJECT_NAME}_node
        PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
)

# 链接库
target_link_libraries(${PROJECT_NAME}_node
        ${OpenCV_LIBS}
        openvino::runtime
        TBB::tbb
)

# ROS2 依赖
ament_target_dependencies(${PROJECT_NAME}_node
        rclcpp
        sensor_msgs
        cv_bridge
        ament_index_cpp
        OpenCV
        auto_aim_interfaces
        visualization_msgs
        image_transport
        tf2_geometry_msgs
        rclcpp_components
)

# 安装可执行文件
install(TARGETS
        ${PROJECT_NAME}_node
        DESTINATION lib/${PROJECT_NAME}
)

# 安装头文件
install(DIRECTORY include/
        DESTINATION include/
)

# 安装模型文件
install(DIRECTORY model/
        DESTINATION share/${PROJECT_NAME}/model
)

# 安装测试文件
install(DIRECTORY test/
        DESTINATION share/${PROJECT_NAME}/test
)

ament_package()