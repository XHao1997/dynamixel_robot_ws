#pragma once

#include <algorithm>
#include <cstdint>
#include <memory>
#include <numeric>
#include <string>
#include <vector>
#include <chrono>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "dynamixel_sdk/dynamixel_sdk.h"
#include "dynamixel_sdk_custom_interfaces/msg/set_position.hpp"
#include "dynamixel_sdk_custom_interfaces/msg/set_multi_position.hpp"
#include "dynamixel_sdk_custom_interfaces/msg/move_joint.hpp"


#include "dynamixel_sdk_examples/motors_parameters.hpp"   // ParamListener + params
// AX-12A: about 300 degrees of motion
constexpr double MIN_RAD = 0.0;
constexpr double MAX_RAD = 300.0 * M_PI / 180.0;  // ≈ 5.23599 rad
class ReadWriteNodeAX12A : public rclcpp::Node {
public:
  using SetPosition = dynamixel_sdk_custom_interfaces::msg::SetPosition;
  using SetMultiPosition = dynamixel_sdk_custom_interfaces::msg::SetMultiPosition;
  using MoveJoint = dynamixel_sdk_custom_interfaces::msg::MoveJoint;

  using milliseconds = std::chrono::milliseconds;

  explicit ReadWriteNodeAX12A();
  ~ReadWriteNodeAX12A() override;

private:
  // ---- Dynamixel Control Table (AX-12A, Protocol 1.0)
  static constexpr int ADDR_TORQUE_ENABLE     = 24;  // 1 byte
  static constexpr int ADDR_GOAL_POSITION     = 30;  // 2 bytes
  static constexpr int ADDR_PRESENT_POSITION  = 36;  // 2 bytes
  static constexpr float PROTOCOL_VERSION     = 1.0f;
  static constexpr int ADDR_MOVING_SPEED       = 32;  // 2 bytes
  // ---- Helpers
  static inline double ticksToRad(uint16_t ticks) {
    // 0..1023 ticks -> 0..300 degrees -> radians
    return (static_cast<double>(ticks) / 1023.0) * (300.0 * M_PI / 180.0);
  }

  // ---- ROS Callbacks
  void onSetPosition(const SetPosition::SharedPtr msg);
  bool onSetPosition(const uint8_t id, const uint16_t goal);
  void onSetMultiPosition(const SetMultiPosition::SharedPtr msg);
  void onSetMultiPosition(const std::vector<double>& positions_rad);
  void setOneMotorTicks(uint8_t id, int pos, int mirror_pos);
  void pollAndPublishJointState();
  // ---- Dynamixel helpers
  bool enableTorque(uint8_t id, bool enable);
  void moveJ(const MoveJoint::SharedPtr msg);
  // ---- Parameters / configuration
  std::string device_name_;
  int baudrate_{57600};
  double poll_rate_hz_{10.0};
  std::vector<uint8_t> dxl_ids_;
  std::vector<int32_t> goal_positions_;
  std::vector<std::string> joint_names_;

  // ---- ROS I/O
  rclcpp::Subscription<SetPosition>::SharedPtr set_position_sub_;
  rclcpp::Subscription<SetMultiPosition>::SharedPtr set_multi_position_sub_;
  rclcpp::Subscription<MoveJoint>::SharedPtr moveJ_sub_;



  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_pub_;
  rclcpp::TimerBase::SharedPtr timer_;

  // ---- Dynamixel SDK handlers (per instance)
  dynamixel::PortHandler*   port_handler_{nullptr};
  dynamixel::PacketHandler* packet_handler_{nullptr};
};
