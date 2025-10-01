//
// Created by rm_autoaim on 2025/10/2.
//

#include "rclcpp/rclcpp.hpp"
#include "../include/rm_serial_driver/rm_serial_driver.hpp"  // 你节点类的头文件路径，注意改成你项目里实际的

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rm_serial_driver::RMSerialDriver>(rclcpp::NodeOptions{});
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
