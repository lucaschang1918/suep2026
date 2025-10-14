cmake_minimum_required(VERSION 3.10)
project(hik_camera)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

add_definitions(-Wall -Werror)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

#Opencv
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)

#Ros
find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

ament_auto_add_library(${PROJECT_NAME} SHARED
        src/hik_camera_node.cpp
)

ament_target_dependencies(${PROJECT_NAME}
        rclcpp
        sensor_msgs
        cv_bridge
        ament_index_cpp
        OpenCV
        camera_info_manager
)

target_include_directories(
        ${PROJECT_NAME}
        PUBLIC

        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/hikSDK/include>
        $<INSTALL_INTERFACE:include>
)

if(CMAKE_SYSTEM_PROCESSOR MATCHES "x86_64")
  target_link_directories(${PROJECT_NAME} PUBLIC hikSDK/lib/amd64)
  install(DIRECTORY hikSDK/lib/amd64/ DESTINATION lib)
elseif(CMAKE_SYSTEM_PROCESSOR MATCHES "aarch64")
  target_link_directories(${PROJECT_NAME} PUBLIC hikSDK/lib/arm64)
  install(DIRECTORY hikSDK/lib/arm64/ DESTINATION lib)
else()
  message(FATAL_ERROR "Unsupported host system architecture: ${CMAKE_HOST_SYSTEM_PROCESSOR}!")
endif()

target_link_libraries(${PROJECT_NAME}
        FormatConversion
        MediaProcess
        MvCameraControl
        MVRender
        MvUsb3vTL
)

rclcpp_components_register_node(${PROJECT_NAME}
        PLUGIN hik_camera::HikCameraNode
        EXECUTABLE ${PROJECT_NAME}_node
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  list(APPEND AMENT_LINT_AUTO_EXCLUDE
          ament_cmake_copyright
          ament_cmake_cpplint
          ament_cmake_uncrustify
  )
  ament_lint_auto_find_test_dependencies()
endif()

ament_auto_package(
        INSTALL_TO_SHARE
        config
        launch
)
