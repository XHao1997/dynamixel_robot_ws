#include "read_write_node.hpp"
#include <cmath>

using namespace std::chrono_literals;

// ============ Constructor ============
ReadWriteNodeAX12A::ReadWriteNodeAX12A()
: rclcpp::Node("read_write_node_ax12a")
{
  // Scan a small range of IDs (0..19)

  // Parameters from generated ParamListener in dynamixel_sdk_examples
  auto config_listener = std::make_shared<dynamixel_sdk_examples::ParamListener>(
    this->get_node_parameters_interface(), this->get_logger());
  auto cfg = config_listener->get_params();

  device_name_ = cfg.device_name;
  baudrate_    = cfg.baudrate;
  dxl_ids_     = std::vector<uint8_t>(cfg.ids.begin(), cfg.ids.end());
  joint_names_ = cfg.joint_names;

  // poll_rate_hz_ = this->declare_parameter<double>("poll_rate_hz", 0.5);
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

  group_sync_write_ = std::make_unique<dynamixel::GroupSyncWrite>(
    port_handler_,
    packet_handler_,
    ADDR_GOAL_POSITION,
    2
  );
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
    uint8_t dxl_error;
    packet_handler_->write2ByteTxRx(port_handler_, id, ADDR_MOVING_SPEED, 200, &dxl_error);
    if (!enableTorque(id, true)) {
      throw std::runtime_error("Torque enable failed");
    }
  }

  // ---- ROS I/O
  auto qos = rclcpp::QoS(rclcpp::KeepLast(qos_depth)).reliable().durability_volatile();

  get_position_srv_ = this->create_service<GetPosition>(
    "get_position",
    std::bind(
      &ReadWriteNodeAX12A::handleGetPosition,
      this,
      std::placeholders::_1,
      std::placeholders::_2)
  );

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

    set_torque_sub_ = this->create_subscription<std_msgs::msg::Int32>(
    "set_torque", qos,
    [this](const std_msgs::msg::Int32::SharedPtr msg)
    {
      if (msg->data == 0) {
        RCLCPP_WARN(this->get_logger(), "/set_torque==0, disabling torque on all motors.");
        for (auto id : dxl_ids_) {
          (void)enableTorque(static_cast<uint8_t>(id), false);
        }
      } 
      else if (msg->data == 1)
      {
        for (auto id : dxl_ids_) {
          (void)enableTorque(static_cast<uint8_t>(id), true);
        }
       {
        RCLCPP_DEBUG(this->get_logger(),
          "Received /set_torque: %d (ignored; only 0 disables torque)", msg->data);
      }
    }
  }
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
    this->packet_handler_ = nullptr;
    this->group_sync_write_ = nullptr;
    this->port_handler_ = nullptr;
  }
  RCLCPP_INFO(get_logger(), "Shutdown read_write_node_ax12a");
}
void ReadWriteNodeAX12A::handleGetPosition(
  const std::shared_ptr<GetPosition::Request>  request,
  std::shared_ptr<GetPosition::Response>       response)
{
  int16_t req_id = request->id;  
  response->positions.clear();

  auto read_one = [this](uint8_t id) -> int32_t {
    uint8_t dxl_error = 0;
    uint16_t present_ticks = 0;

    int dxl_comm_result = packet_handler_->read2ByteTxRx(
      port_handler_,
      id,
      ADDR_PRESENT_POSITION,
      &present_ticks,
      &dxl_error
    );

    if (dxl_comm_result != COMM_SUCCESS) {
      RCLCPP_ERROR(
        this->get_logger(),
        "GetPosition: read failed ID:%u (%s)",
        id, packet_handler_->getTxRxResult(dxl_comm_result));
      return -1;  
    }

    if (dxl_error != 0) {
      RCLCPP_ERROR(
        this->get_logger(),
        "GetPosition: DXL error on ID:%u (%s)",
        id, packet_handler_->getRxPacketError(dxl_error));
      return -1;
    }

    return static_cast<int32_t>(present_ticks);  // 0..1023
  };

  if (req_id == -1) {
    // 返回所有关节位置，顺序与 dxl_ids_ 一致
    response->positions.reserve(dxl_ids_.size());
    for (uint8_t id : dxl_ids_) {
      uint16_t pos = read_one(id);
      if (id!=3 && id!=5){
          if (id==4){
              response->positions.push_back(ticksToRad(pos)-JOINT3_OFFSET);

          }else{
          response->positions.push_back(ticksToRad(pos));
      }
      sleep(0.01);
    }}

    RCLCPP_DEBUG(
      this->get_logger(),
      "GetPosition: all joints -> %zu values",
      response->positions.size());
  } else {
    uint8_t id = static_cast<uint8_t>(req_id);  // 这里才转成 uint8_t
    uint16_t pos = read_one(id);
    response->positions.push_back(ticksToRad(static_cast<uint16_t>(pos)));
    if (id==4){
        response->positions[0]=response->positions[0]-JOINT3_OFFSET;
    }
    else if (id==5)
    {
       response->positions[0]=response->positions[0]+JOINT3_OFFSET;
    }
    
    RCLCPP_DEBUG(
      this->get_logger(),
      "GetPosition: ID:%u -> %f",
      id, ticksToRad(pos));
  }
}



// ============ SetPosition callback ============
void ReadWriteNodeAX12A::onSetPosition(const SetPosition::SharedPtr msg) {
  const uint8_t id = static_cast<uint8_t>(msg->id);

  uint8_t dxl_error = 0;
  // Clamp the integer position field to the valid range
  // int pos = static_cast<int>(msg->position);
  // pos = std::clamp(pos, 0, 1023);
  // const uint16_t goal = static_cast<uint16_t>(pos);
  
  int dxl_comm_result = onSetPosition(id, msg->position, msg->if_rad);

  if (dxl_comm_result != COMM_SUCCESS) {
    RCLCPP_WARN(this->get_logger(), "TX/RX failed: %s",
                packet_handler_->getTxRxResult(dxl_comm_result));
  } else if (dxl_error != 0) {
    RCLCPP_WARN(this->get_logger(), "DXL error: %s",
                packet_handler_->getRxPacketError(dxl_error));
  } else {
    RCLCPP_DEBUG(this->get_logger(), "Set [ID:%u] Goal Position: %u", id, msg->position);
  }	
}

bool ReadWriteNodeAX12A::onSetPosition(const uint8_t id, const double goal,  bool if_rad) {
	int   dxl_comm_result;
  if (if_rad == false)
  {
    uint8_t dxl_error = 0;
    RCLCPP_INFO(
      this->get_logger(),
      "Setting ID:%u Goal Position: %u",
      id, goal);
    // packet_handler_->write2ByteTxRx(port_handler_, id, ADDR_MOVING_SPEED, 200, &dxl_error);
      dxl_comm_result = packet_handler_->write2ByteTxRx(
      port_handler_, id, ADDR_GOAL_POSITION, goal, &dxl_error);
  }
  else
  {
    uint16_t goal_ticks = radToTicks(static_cast<double>(goal));
    uint8_t dxl_error = 0;
    RCLCPP_INFO(
      this->get_logger(),
      "Setting ID:%u Goal Position (rad): %f -> ticks: %f",
      id, goal, goal_ticks);
    if (id==2)
    {
      uint16_t goal_ticks_mirror = radToTicks(-static_cast<double>(goal));
      dxl_comm_result = packet_handler_->write2ByteTxRx(
      port_handler_, id, ADDR_GOAL_POSITION, goal_ticks, &dxl_error);
      dxl_comm_result = packet_handler_->write2ByteTxRx(
      port_handler_, id+1, ADDR_GOAL_POSITION, goal_ticks_mirror, &dxl_error);

    }
    else if (id==3)
    {
      double goal_new = goal + JOINT3_OFFSET;
      RCLCPP_DEBUG(
      this->get_logger(),
      "Mirroring for ID 3 with offset: %f -> %f",
      goal, goal_new);
      uint16_t goal_ticks = radToTicks(static_cast<double>(goal_new));
      uint16_t goal_ticks_mirror = radToTicks(-static_cast<double>(goal_new));
      dxl_comm_result = packet_handler_->write2ByteTxRx(
      port_handler_, id+1, ADDR_GOAL_POSITION, goal_ticks, &dxl_error);
      dxl_comm_result = packet_handler_->write2ByteTxRx(
      port_handler_, id+2, ADDR_GOAL_POSITION, goal_ticks_mirror, &dxl_error);
    }
    else
    {
      RCLCPP_INFO(
      this->get_logger(),
      "Setting ID:%u Goal Position: %u",
      id, goal_ticks);
          // packet_handler_->write2ByteTxRx(port_handler_, id, ADDR_MOVING_SPEED, 200, &dxl_error);
      dxl_comm_result = packet_handler_->write2ByteTxRx(
      port_handler_, id, ADDR_GOAL_POSITION, goal_ticks, &dxl_error);
    }
  }

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


void ReadWriteNodeAX12A::moveJ(const MoveJoint::SharedPtr msg)
{
  RCLCPP_INFO(this->get_logger(), "Received MoveJ command");

  if (!group_sync_write_) {
    RCLCPP_ERROR(this->get_logger(), "GroupSyncWrite not initialized!");
    return;
  }

  if (msg->joint_positions.size() != 6)  {
    RCLCPP_WARN_THROTTLE(
      this->get_logger(), *this->get_clock(), 5000,
      "Mismatched joint count: expected 6, got %zu", msg->joint_positions.size());
    return;
  }

  // 0) convert all radians -> ticks
  std::vector<int> ticks;
  for (size_t i = 0; i < msg->joint_positions.size(); ++i) {
    double rad;
    rad = msg->joint_positions[i];
    if (i == 2) {
      // For IDs 3 and 5, mirror the position
      rad = rad+JOINT3_OFFSET;
    }

    ticks.push_back(radToTicks(rad)); 

    if(i == 1 || i == 2) {
      // For IDs 3 and 5, mirror the position
      ticks.push_back(radToTicks(-rad)); 
      
    }
    
  }
  // 2) log ticks for debugging
  RCLCPP_INFO(this->get_logger(), "Converted ticks:");
  for (size_t i = 0; i < dxl_ids_.size() && i < ticks.size(); ++i) {
    RCLCPP_INFO(this->get_logger(), "  ID:%u -> ticks:%d",
                static_cast<unsigned int>(dxl_ids_[i]), ticks[i]);
  }
  if (ticks.size() != dxl_ids_.size()) {
    RCLCPP_WARN(this->get_logger(),
                "ticks vector size (%zu) != dxl_ids_ size (%zu)",
                ticks.size(), dxl_ids_.size());
  }

  // 3) pack into GroupSyncWrite
  group_sync_write_->clearParam();

  for (size_t i = 0; i < dxl_ids_.size(); ++i) {
    uint8_t id = dxl_ids_[i];
    uint16_t goal = static_cast<uint16_t>(std::clamp(ticks[i], 0, 1023));

    uint8_t param_goal[2] = {
      DXL_LOBYTE(goal),
      DXL_HIBYTE(goal)
    };

    if (!group_sync_write_->addParam(id, param_goal)) {
      RCLCPP_ERROR(
        this->get_logger(),
        "Failed to add param to GroupSyncWrite for ID:%u",
        id);
    }
  }

  int dxl_comm_result = group_sync_write_->txPacket();
  if (dxl_comm_result != COMM_SUCCESS) {
    RCLCPP_ERROR(
      this->get_logger(),
      "GroupSyncWrite txPacket failed: %s",
      packet_handler_->getTxRxResult(dxl_comm_result));
  } else {
    RCLCPP_DEBUG(this->get_logger(), "GroupSyncWrite txPacket OK.");
  }

  group_sync_write_->clearParam();  // optional
}


int ReadWriteNodeAX12A::radToTicks(double rad) const
{
  // Clamp to your AX-12A motion range
  double r = std::clamp(rad, MIN_RAD, MAX_RAD);

  const double ratio = (r - MIN_RAD) / (MAX_RAD - MIN_RAD);
  int tick = static_cast<int>(std::round(ratio * 1023.0));

  return std::clamp(tick, 0, 1023);
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

