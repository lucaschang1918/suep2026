cmake_minimum_required(VERSION 3.10)
project(armor_tracker)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

add_definitions(-Wall -Werror)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

ament_auto_add_library(${PROJECT_NAME} SHARED
        DIRECTORY src
)

# --- 头文件目录 ---
target_include_directories(${PROJECT_NAME} PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include/armor_tracker>
        ${OpenCV_INCLUDE_DIRS}
        ${EIGEN3_INCLUDE_DIRS}
)

rclcpp_components_register_node(${PROJECT_NAME}
        PLUGIN rm_auto_aim::ArmorTrackerNode
        EXECUTABLE ${PROJECT_NAME}_node
)

ament_auto_package()