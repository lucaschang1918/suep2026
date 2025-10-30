cmake_minimum_required(VERSION 3.10)
project(rm_serial_driver)

# C++14
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

add_compile_options(-Wall -Werror)

#测试使用
find_package(OpenCV REQUIRED)

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

ament_auto_add_library(${PROJECT_NAME} SHARED
#        src/rm_serial_driver.cpp
        src/fake.cpp
        src/crc.cpp
)

target_include_directories(${PROJECT_NAME} PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include/rm_serial_driver>
        $<INSTALL_INTERFACE:include>
        ${OpenCV_INCLUDE_DIRS}
)

# 测试使用
target_link_libraries(${PROJECT_NAME}
        ${OpenCV_LIBS}
)

rclcpp_components_register_node(${PROJECT_NAME}

#        PLUGIN rm_serial_driver::RMSerialDriver

        PLUGIN rm_serial_driver::FakeSerialDriver  # 测试使用

        EXECUTABLE ${PROJECT_NAME}_node
)

ament_auto_package(
        INSTALL_TO_SHARE
        config
        launch
)