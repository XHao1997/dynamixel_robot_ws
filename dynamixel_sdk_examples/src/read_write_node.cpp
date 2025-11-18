#include "read_write_node.hpp"
#include <cmath>

using namespace std::chrono_literals;

// ============ Constructor ============
ReadWriteNodeAX12A::ReadWriteNodeAX12A()
: rclcpp::Node("read_write_node_ax12a")
{
  // Parameters from generated ParamListener in dynamixel_sdk_examples
  auto config_listener = std::make_shared<dynamixel_sdk_examples::ParamListener>(
    this->get_node_parameters_interface(), this->get_logger());
  auto cfg = config_listener->get_params();

  device_name_ = cfg.device_name;
  baudrate_    = cfg.baudrate;
  dxl_ids_     = std::vector<uint8_t>(cfg.ids.begin(), cfg.ids.end());
  joint_names_ = cfg.joint_names;

  poll_rate_hz_ = this->declare_parameter<double>("poll_rate_hz", 2);
  const int qos_depth = this->declare_parameter<int>("qos_depth", 1);

  RCLCPP_INFO(this->get_logger(), "Device: %s  Baud: %d", device_name_.c_str(), baudrate_);
  RCLCPP_INFO(this->get_logger(), "IDs: %s",
    std::accumulate(dxl_ids_.begin(), dxl_ids_.end(), std::string(),
      [](const std::string& a, uint8_t b){
        return a.empty() ? std::to_string(b) : a + ", " + std::to_string(b);
      }).c_str());
  RCLCPP_INFO(this->get_logger(), "Joint names: %s",
    std::accumulate(joint_names_.begin(), joint_names_.end(), std::string(),
      [](const std::string& a, const std::string& b){
        return a.empty() ? b : a + ", " + b;
      }).c_str());

  if (dxl_ids_.empty()) {
    throw std::runtime_error("No Dynamixel IDs provided.");
  }

  // ---- Open serial
  port_handler_   = dynamixel::PortHandler::getPortHandler(device_name_.c_str());
  packet_handler_ = dynamixel::PacketHandler::getPacketHandler(PROTOCOL_VERSION);

  if (!port_handler_->openPort()) {
    RCLCPP_FATAL(get_logger(), "Failed to open port %s", device_name_.c_str());
    throw std::runtime_error("openPort failed");
  }
  if (!port_handler_->setBaudRate(baudrate_)) {
    RCLCPP_FATAL(get_logger(), "Failed to set baudrate %d", baudrate_);
    throw std::runtime_error("setBaudRate failed");
  }
  RCLCPP_INFO(get_logger(), "Serial ready.");

  // ---- Enable torque on all IDs
  for (auto id : dxl_ids_) {
    if (!enableTorque(id, true)) {
      throw std::runtime_error("Torque enable failed");
    }
  }

  // ---- ROS I/O
  auto qos = rclcpp::QoS(rclcpp::KeepLast(qos_depth)).reliable().durability_volatile();

  joint_pub_ = this->create_publisher<sensor_msgs::msg::JointState>("joint_states", qos);

  set_position_sub_ = this->create_subscription<SetPosition>(
    "set_position", qos,
    [this](const SetPosition::SharedPtr msg) { this->onSetPosition(msg); }
  );

  set_multi_position_sub_ = this->create_subscription<SetMultiPosition>(
    "set_multi_position", qos,
    [this](const SetMultiPosition::SharedPtr msg) { this->onSetMultiPosition(msg); }
  );
  
  moveJ_sub_ = this->create_subscription<MoveJoint>(
    "move_joint", qos,
    [this](const MoveJoint::SharedPtr msg) { this->moveJ(msg); }
  );

  // ---- Polling timer
  auto period = std::chrono::duration<double>(1.0 / std::max(1e-3, poll_rate_hz_));
  timer_ = this->create_wall_timer(
    std::chrono::duration_cast<std::chrono::milliseconds>(period),
    std::bind(&ReadWriteNodeAX12A::pollAndPublishJointState, this)
  );

  RCLCPP_INFO(get_logger(), "read_write_node_ax12a started.");
}

// ============ Destructor ============
ReadWriteNodeAX12A::~ReadWriteNodeAX12A() {
  if (packet_handler_ && port_handler_) {
    for (auto id : dxl_ids_) {
      (void)enableTorque(id, false);
    }
    port_handler_->closePort();
  }
  RCLCPP_INFO(get_logger(), "Shutdown read_write_node_ax12a");
}

// ============ SetPosition callback ============
void ReadWriteNodeAX12A::onSetPosition(const SetPosition::SharedPtr msg) {
  const uint8_t id = static_cast<uint8_t>(msg->id);

  uint8_t dxl_error = 0;
  // Clamp the integer position field to the valid range
  int pos = static_cast<int>(msg->position);
  pos = std::clamp(pos, 0, 1023);
  const uint16_t goal = static_cast<uint16_t>(pos);

  int dxl_comm_result = onSetPosition(id, goal);

  if (dxl_comm_result != COMM_SUCCESS) {
    RCLCPP_WARN(this->get_logger(), "TX/RX failed: %s",
                packet_handler_->getTxRxResult(dxl_comm_result));
  } else if (dxl_error != 0) {
    RCLCPP_WARN(this->get_logger(), "DXL error: %s",
                packet_handler_->getRxPacketError(dxl_error));
  } else {
    RCLCPP_DEBUG(this->get_logger(), "Set [ID:%u] Goal Position: %u", id, goal);
  }	
}

bool ReadWriteNodeAX12A::onSetPosition(const uint8_t id, const uint16_t goal) {
	uint8_t dxl_error = 0;
  RCLCPP_INFO(
    this->get_logger(),
    "Setting ID:%u Goal Position: %u",
    id, goal);
	// packet_handler_->write2ByteTxRx(port_handler_, id, ADDR_MOVING_SPEED, 200, &dxl_error);
	int dxl_comm_result = packet_handler_->write2ByteTxRx(
		port_handler_, id, ADDR_GOAL_POSITION, goal, &dxl_error);
	return dxl_comm_result;
}

// Inside your class:
void ReadWriteNodeAX12A::onSetMultiPosition(const std::vector<double>& positions_rad)
{
  // Assume dxl_ids_ already holds the motor IDs in the same order as positions_rad
  if (positions_rad.size() != dxl_ids_.size()) {
    RCLCPP_WARN(this->get_logger(),
                "Mismatched id and positions_rad sizes: %zu vs %zu",
                dxl_ids_.size(), positions_rad.size());
    return;
  }

  if (positions_rad.empty()) {
    RCLCPP_WARN(this->get_logger(), "Empty positions_rad in onSetMultiPosition");
    return;
  }

  // Lambda: rad -> 0..1023 (clamped)
  auto rad_to_tick = [](double rad) {
    constexpr double MIN_R = MIN_RAD;
    constexpr double MAX_R = MAX_RAD;

    // Clamp radians into valid range
    rad = std::clamp(rad, MIN_R, MAX_R);

    // Normalize to [0, 1]
    const double ratio = (rad - MIN_R) / (MAX_R - MIN_R);

    // Scale to [0, 1023]
    int tick = static_cast<int>(std::lround(ratio * 1023.0));
    return std::clamp(tick, 0, 1023);
  };

  // Lambda that handles ONE motor (same idea as before, now from radians)
  auto set_one_motor = [this, &rad_to_tick](uint8_t id,
                                            double rad_pos,
                                            double mirror_rad_pos)
  {
    // Convert main position
    int pos = rad_to_tick(rad_pos);

    // Special case for ID 3 and 5: mirrored motion
    if (id == 3 || id == 5) {
      int mirror_tick = rad_to_tick(mirror_rad_pos);
      pos = 1023 - mirror_tick;
    }

    const uint16_t goal = static_cast<uint16_t>(pos);

    int dxl_comm_result = onSetPosition(id, goal);
    if (dxl_comm_result != COMM_SUCCESS) {
      RCLCPP_WARN(this->get_logger(),
                  "TX/RX failed for ID:%u (%s)",
                  id, packet_handler_->getTxRxResult(dxl_comm_result));
    }
  };

  // Loop over all joints and call the lambda
  for (size_t i = 0; i < dxl_ids_.size(); ++i) {
    uint8_t id = static_cast<uint8_t>(dxl_ids_[i]);
    double rad_pos = positions_rad[i];

    double mirror_rad = rad_pos;
    this->setOneMotorTicks(id, rad_pos, mirror_rad);
  }
}


void ReadWriteNodeAX12A::setOneMotorTicks(uint8_t id, int pos, int mirror_pos)
{
  int clamped_pos = std::clamp(pos, 0, 1023);

  if (id == 3 || id == 5) {
    clamped_pos = 1023 - std::clamp(mirror_pos, 0, 1023);
  }

  const uint16_t goal = static_cast<uint16_t>(clamped_pos);

  int dxl_comm_result = this->onSetPosition(id, goal);
  if (dxl_comm_result != COMM_SUCCESS) {
    RCLCPP_WARN(this->get_logger(),
                "TX/RX failed for ID:%u (%s)",
                id, packet_handler_->getTxRxResult(dxl_comm_result));
  }
}

void ReadWriteNodeAX12A::onSetMultiPosition(const SetMultiPosition::SharedPtr msg)
{
  // 1) Handle torque disable case
  if (!msg->torque_enable) {
    std::for_each(dxl_ids_.begin(), dxl_ids_.end(),
                  [this](int id) {
                    enableTorque(static_cast<uint8_t>(id), false);
                  });
    return;
  }
  // 2) Copy msg data into member variables (if you really need them as members)
  dxl_ids_        = msg->ids;
  goal_positions_ = msg->positions;

  RCLCPP_INFO(
    this->get_logger(),
    "Received SetMultiPosition for %zu motors",
    dxl_ids_.size());

  if (dxl_ids_.size() != goal_positions_.size()) {
    RCLCPP_WARN(this->get_logger(),
                "Mismatched id and position array sizes: %zu vs %zu",
                dxl_ids_.size(), goal_positions_.size());
    return;
  }

  if (dxl_ids_.empty()) {
    RCLCPP_WARN(this->get_logger(), "Empty id array in SetMultiPosition");
    return;
  }

  for (size_t i = 0; i < dxl_ids_.size(); ++i) {
    uint8_t id        = static_cast<uint8_t>(dxl_ids_[i]);
    int     raw_pos   = static_cast<int>(goal_positions_[i]);
    int mirror_source = raw_pos;

    this->setOneMotorTicks(id, raw_pos, mirror_source);
  }
}

// ============ Enable/disable torque ============
bool ReadWriteNodeAX12A::enableTorque(uint8_t id, bool enable) {
  uint8_t dxl_error = 0;
  int dxl_comm_result = packet_handler_->write1ByteTxRx(
    port_handler_, id, ADDR_TORQUE_ENABLE, enable ? 1 : 0, &dxl_error);

  if (dxl_comm_result != COMM_SUCCESS) {
    RCLCPP_ERROR(this->get_logger(), "Torque %s failed: %s",
                 enable ? "enable" : "disable",
                 packet_handler_->getTxRxResult(dxl_comm_result));
    return false;
  }
  if (dxl_error) {
    RCLCPP_ERROR(this->get_logger(), "Torque %s error: %s",
                 enable ? "enable" : "disable",
                 packet_handler_->getRxPacketError(dxl_error));
    return false;
  }
  RCLCPP_INFO(this->get_logger(), "Torque %s OK (ID:%u)",
              enable ? "enabled" : "disabled", id);
  return true;
}

// ============ Poll & publish ============
void ReadWriteNodeAX12A::pollAndPublishJointState() {
  std::vector<double> positions_rad;
  positions_rad.reserve(dxl_ids_.size());

  uint8_t dxl_error = 0;

  for (auto id : dxl_ids_) {
    uint16_t present_ticks = 0;
    int dxl_comm_result = packet_handler_->read2ByteTxRx(
      port_handler_, id, ADDR_PRESENT_POSITION, &present_ticks, &dxl_error);

    if (dxl_comm_result != COMM_SUCCESS) {
      RCLCPP_WARN_THROTTLE(get_logger(), *this->get_clock(), 2000,
                           "Read ID:%u failed: %s", id, packet_handler_->getTxRxResult(dxl_comm_result));
      positions_rad.push_back(std::numeric_limits<double>::quiet_NaN());
      continue;
    }
    if (dxl_error != 0) {
      RCLCPP_WARN_THROTTLE(get_logger(), *this->get_clock(), 2000,
                           "DXL error on ID:%u: %s", id, packet_handler_->getRxPacketError(dxl_error));
      positions_rad.push_back(std::numeric_limits<double>::quiet_NaN());
      continue;
    }

    positions_rad.push_back(ticksToRad(present_ticks));
  }

  // names length = ids length
  std::vector<std::string> names;
  if (joint_names_.size() == dxl_ids_.size()) {
    names = joint_names_;
  } else {
    names.reserve(dxl_ids_.size());
    for (auto id : dxl_ids_) names.emplace_back("dxl_" + std::to_string(id));
    if (!joint_names_.empty()) {
      RCLCPP_WARN_THROTTLE(get_logger(), *this->get_clock(), 5000,
        "joint_names count (%zu) != ids count (%zu). Using auto names.",
        joint_names_.size(), dxl_ids_.size());
    }
  }

  sensor_msgs::msg::JointState js;
  js.header.stamp = this->now();
  js.name = std::move(names);
  js.position = std::move(positions_rad);
  joint_pub_->publish(js);
}


void ReadWriteNodeAX12A::moveJ(const MoveJoint::SharedPtr msg){
  RCLCPP_INFO(this->get_logger(), "Received MoveJ command");
  if (msg->joint_positions.size() != 6) {
    RCLCPP_WARN_THROTTLE(this->get_logger(),*this->get_clock(), 5000,
                "Mismatched joint_names and positions array sizes: required 6 but got %zu",
                msg->joint_positions.size());
    return;
  }


}

static volatile std::sig_atomic_t g_stop = 0;
void sigintHandler(int) { g_stop = 1; }

int main(int argc, char* argv[]) {
  std::signal(SIGINT, sigintHandler);
  rclcpp::init(argc, argv);

  auto node = std::make_shared<ReadWriteNodeAX12A>();
  rclcpp::executors::SingleThreadedExecutor exec;
  exec.add_node(node);

  while (rclcpp::ok() && !g_stop) {
    exec.spin_some(std::chrono::milliseconds(5));
  }

  exec.cancel();
  exec.remove_node(node);
  node.reset();
  rclcpp::shutdown();
  return 0;
}

