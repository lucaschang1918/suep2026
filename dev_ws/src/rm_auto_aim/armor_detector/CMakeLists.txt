cmake_minimum_required(VERSION 3.10)

project(armor_detector)

# C++20
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake_auto REQUIRED)
find_package(OpenCV REQUIRED)
ament_auto_find_build_dependencies()

# OpenVINO
find_package(OpenVINO REQUIRED)
find_package(TBB REQUIRED)

ament_auto_add_library(${PROJECT_NAME} SHARED
        DIRECTORY src
)

target_include_directories(${PROJECT_NAME}
        PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        ${OpenCV_INCLUDE_DIRS}
)

# 链接库
target_link_libraries(${PROJECT_NAME}
        ${OpenCV_LIBS}
        openvino::runtime
        TBB::tbb
)

rclcpp_components_register_node(${PROJECT_NAME}
        PLUGIN rm_auto_aim::ArmorDetectorNode
        EXECUTABLE armor_detector_node
)

#############
## Install ##
#############
ament_auto_package(
        INSTALL_TO_SHARE
        model
        test
)
