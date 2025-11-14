#include <algorithm>
#include <cstdint>
#include <cstdio>
#include <csignal>
#include <memory>
#include <string>
#include <vector>
#include <chrono>
#include "rclcpp/rclcpp.hpp"
#include "dynamixel_sdk_examples/motors_parameters.hpp"  // generated header

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<rclcpp::Node>("test_parameter_load");

    // The ParamListener automatically reads from the node’s declared parameters
    auto param_listener = std::make_shared<dynamixel_sdk_examples::ParamListener>(node);
    auto params = param_listener->get_params();

    // Print to verify that parameters are loaded correctly
    RCLCPP_INFO(node->get_logger(), "Parameters loaded:");
    // Print motor1 parameters generated from motors.yaml

    // RCLCPP_INFO(node->get_logger(), "update_rate_hz: %.2f", params.update_rate_hz);
    // RCLCPP_INFO(node->get_logger(), "frame_id: %s", params.frame_id.c_str());
    rclcpp::sleep_for(std::chrono::milliseconds(100));
    rclcpp::shutdown();
    return 0;
}
