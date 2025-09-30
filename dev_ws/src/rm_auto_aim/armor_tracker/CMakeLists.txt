cmake_minimum_required(VERSION 3.10)
project(armor_tracker)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

add_definitions(-Wall -Werror)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# --- ROS2 基础依赖 ---
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(image_transport REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(auto_aim_interfaces REQUIRED)
find_package(angles REQUIRED)
find_package(std_srvs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(message_filters REQUIRED)

# --- Eigen / OpenCV ---
find_package(Eigen3 REQUIRED)
find_package(OpenCV REQUIRED)

# --- 库 ---
add_library(${PROJECT_NAME} SHARED
        src/extended_kalman_filter.cpp
        src/tracker.cpp
        src/tracker_node.cpp
        ../../rm_serial_driver/src/crc.cpp
        ../../rm_serial_driver/src/rm_serial_driver.cpp
        ../../rm_serial_driver/include/rm_serial_driver/rm_serial_driver.hpp
        ../../rm_serial_driver/include/rm_serial_driver/crc.hpp
        ../../rm_serial_driver/include/rm_serial_driver/packet.hpp
)

# --- 依赖 ---
ament_target_dependencies(${PROJECT_NAME}
        rclcpp
        rclcpp_components
        sensor_msgs
        cv_bridge
        ament_index_cpp
        OpenCV
        auto_aim_interfaces
        visualization_msgs
        image_transport
        tf2_geometry_msgs
        angles
        std_srvs
        geometry_msgs
        message_filters
)

# --- 头文件目录 ---
target_include_directories(${PROJECT_NAME} PUBLIC
        include
        ${OpenCV_INCLUDE_DIRS}
        ${EIGEN3_INCLUDE_DIRS}
)

# --- 注册节点 ---
rclcpp_components_register_node(${PROJECT_NAME}
        PLUGIN rm_auto_aim::ArmorTrackerNode
        EXECUTABLE ${PROJECT_NAME}_node
)

# --- 导出给其他包使用 ---
ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
ament_export_dependencies(
        rclcpp
        rclcpp_components
        sensor_msgs
        cv_bridge
        ament_index_cpp
        OpenCV
        auto_aim_interfaces
        visualization_msgs
        image_transport
        tf2_geometry_msgs
        angles
        std_srvs
        geometry_msgs
        message_filters
)

# --- 安装 ---
install(
        TARGETS ${PROJECT_NAME}
        EXPORT export_${PROJECT_NAME}
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION bin
)

install(
        DIRECTORY include/
        DESTINATION include
)

install(
        TARGETS ${PROJECT_NAME}_node
        DESTINATION lib/${PROJECT_NAME}
)

ament_package()