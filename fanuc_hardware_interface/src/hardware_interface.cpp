// SPDX-FileCopyrightText: 2025, FANUC America Corporation
// SPDX-FileCopyrightText: 2025, FANUC CORPORATION
//
// SPDX-License-Identifier: Apache-2.0

#include "fanuc_robot_driver/hardware_interface.hpp"

#include <chrono>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include "fanuc_client/gpio_buffer.hpp"
#include "fanuc_robot_driver/constants.hpp"
#include "gpio_config/gpio_config.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/logging.hpp"

namespace fanuc_robot_driver
{
namespace
{
using CommandGPIOTypes = ::fanuc_client::GPIOBuffer::CommandGPIOTypes;
using StatusGPIOTypes = ::fanuc_client::GPIOBuffer::StatusGPIOTypes;

constexpr auto kFRHWInterface = "FR_HW_Interface";
constexpr int kNumberConnectionAttempts = 5;

int StringToInt(const std::string& param_name, const std::string& param_value)
{
  try
  {
    return std::stoi(param_value);
  }
  catch (const std::invalid_argument& e)
  {
    RCLCPP_ERROR(rclcpp::get_logger(kFRHWInterface), "Invalid integer parameter `%s` for parameter named `%s`",
                 param_value.c_str(), param_name.c_str());
    throw;
  }
  catch (const std::out_of_range& e)
  {
    RCLCPP_ERROR(rclcpp::get_logger(kFRHWInterface), "Integer parameter out of range: `%s` for parameter named `%s`",
                 param_value.c_str(), param_name.c_str());
    throw;
  }
}
}  // namespace

struct IOCommandInterface
{
  // Get the state interface name;
  virtual std::string name() const = 0;
  // Update internals based on value.
  virtual void updateBuffer() = 0;
  // The GPIO index.
  uint32_t index;
  // The value as a double (set by ROS 2 control).
  double value = 0;
};

template <CommandGPIOTypes type, typename T>
struct IOCommand : IOCommandInterface
{
  IOCommand(int i, fanuc_client::GPIOBuffer::CommandBlock<type, T>& b) : block(b)
  {
    index = i;
  }

  std::string name() const override
  {
    return std::string(block.type());
  }

  void updateBuffer() override
  {
    block.set(index, static_cast<T>(value));
  }

  // The block in the GPIO command buffer that this index belongs to.
  fanuc_client::GPIOBuffer::CommandBlock<type, T>& block;
};

struct IOStateInterface
{
  virtual std::string name() const = 0;
  // Update value based on internal sources.
  virtual void updateValue() = 0;
  // The GPIO index.
  uint32_t index;
  // The value as a double (read by ROS 2 control).
  double value = 0;
};

template <StatusGPIOTypes type, typename T>
struct IOState : IOStateInterface
{
  IOState(int i, fanuc_client::GPIOBuffer::StatusBlock<type, T>& b) : block(b)
  {
    index = i;
  }

  std::string name() const override
  {
    return std::string(block.type());
  }

  void updateValue() override
  {
    value = static_cast<double>(block.get(index));
  }

  // The block in the GPIO status buffer that this index belongs to.
  fanuc_client::GPIOBuffer::StatusBlock<type, T>& block;
};

struct IOInterfaces
{
  std::vector<std::unique_ptr<IOCommandInterface>> commands;
  std::vector<std::unique_ptr<IOStateInterface>> states;
  fanuc_client::GPIOBuffer buffer;
};

namespace
{

template <CommandGPIOTypes type>
void AppendBoolCmdInterfaces(uint32_t start, uint32_t length, fanuc_client::GPIOBuffer::Builder& builder,
                             std::vector<std::unique_ptr<IOCommandInterface>>& interfaces)
{
  auto& block = builder.addCommandConfig<type, bool>(start, length);
  for (uint32_t index = start; index < start + length; ++index)
  {
    interfaces.emplace_back(std::make_unique<IOCommand<type, bool>>(index, block));
  }
}

template <StatusGPIOTypes type>
void AppendBoolStateInterfaces(uint32_t start, uint32_t length, fanuc_client::GPIOBuffer::Builder& builder,
                               std::vector<std::unique_ptr<IOStateInterface>>& interfaces)
{
  auto& block = builder.addStatusConfig<type, bool>(start, length);
  for (uint32_t index = start; index < start + length; ++index)
  {
    interfaces.emplace_back(std::make_unique<IOState<type, bool>>(index, block));
  }
}

template <CommandGPIOTypes type>
void AppendAnalogCmdInterfaces(uint32_t start, uint32_t length, fanuc_client::GPIOBuffer::Builder& builder,
                               std::vector<std::unique_ptr<IOCommandInterface>>& interfaces)
{
  auto& block = builder.addCommandConfig<type, uint16_t>(start, length);
  for (uint32_t index = start; index < start + length; ++index)
  {
    interfaces.emplace_back(std::make_unique<IOCommand<type, uint16_t>>(index, block));
  }
}

template <StatusGPIOTypes type>
void AppendAnalogStateInterfaces(uint32_t start, uint32_t length, fanuc_client::GPIOBuffer::Builder& builder,
                                 std::vector<std::unique_ptr<IOStateInterface>>& interfaces)
{
  auto& block = builder.addStatusConfig<type, uint16_t>(start, length);
  for (uint32_t index = start; index < start + length; ++index)
  {
    interfaces.emplace_back(std::make_unique<IOState<type, uint16_t>>(index, block));
  }
}

IOInterfaces GPIOConfigToInterfaces(const gpio_config::GPIOTopicConfig& config)
{
  std::vector<std::unique_ptr<IOCommandInterface>> commands;
  std::vector<std::unique_ptr<IOStateInterface>> states;
  fanuc_client::GPIOBuffer::Builder buf_builder{};

  using ::gpio_config::BoolIOCmdType;
  if (config.io_cmd.has_value())
  {
    for (const gpio_config::BoolIOCmdConfig& bool_cmd : *config.io_cmd)
    {
      switch (bool_cmd.type)
      {
        case BoolIOCmdType::DO:
          AppendBoolCmdInterfaces<CommandGPIOTypes::DO>(bool_cmd.start, bool_cmd.length, buf_builder, commands);
          break;
        case BoolIOCmdType::RO:
          AppendBoolCmdInterfaces<CommandGPIOTypes::RO>(bool_cmd.start, bool_cmd.length, buf_builder, commands);
          break;
        case BoolIOCmdType::F:
          AppendBoolCmdInterfaces<CommandGPIOTypes::F>(bool_cmd.start, bool_cmd.length, buf_builder, commands);
          break;
      }
    }
  }

  using ::gpio_config::BoolIOStateType;
  if (config.io_state.has_value())
  {
    for (const gpio_config::BoolIOStateConfig& bool_state : *config.io_state)
    {
      switch (bool_state.type)
      {
        case BoolIOStateType::DO:
          AppendBoolStateInterfaces<StatusGPIOTypes::DO>(bool_state.start, bool_state.length, buf_builder, states);
          break;
        case BoolIOStateType::DI:
          AppendBoolStateInterfaces<StatusGPIOTypes::DI>(bool_state.start, bool_state.length, buf_builder, states);
          break;
        case BoolIOStateType::RO:
          AppendBoolStateInterfaces<StatusGPIOTypes::RO>(bool_state.start, bool_state.length, buf_builder, states);
          break;
        case BoolIOStateType::RI:
          AppendBoolStateInterfaces<StatusGPIOTypes::RI>(bool_state.start, bool_state.length, buf_builder, states);
          break;
        case BoolIOStateType::F:
          AppendBoolStateInterfaces<StatusGPIOTypes::F>(bool_state.start, bool_state.length, buf_builder, states);
          break;
      }
    }
  }

  using ::gpio_config::AnalogIOCmdType;
  if (config.analog_io_cmd.has_value())
  {
    for (const gpio_config::AnalogIOCmdConfig& analog_cmd : *config.analog_io_cmd)
    {
      if (analog_cmd.type == AnalogIOCmdType::AO)
      {
        AppendAnalogCmdInterfaces<CommandGPIOTypes::AO>(analog_cmd.start, analog_cmd.length, buf_builder, commands);
      }
    }
  }

  using ::gpio_config::AnalogIOStateType;
  if (config.analog_io_state.has_value())
  {
    for (const gpio_config::AnalogIOStateConfig& analog_state : *config.analog_io_state)
    {
      switch (analog_state.type)
      {
        case AnalogIOStateType::AO:
          AppendAnalogStateInterfaces<StatusGPIOTypes::AO>(analog_state.start, analog_state.length, buf_builder, states);
          break;
        case AnalogIOStateType::AI:
          AppendAnalogStateInterfaces<StatusGPIOTypes::AI>(analog_state.start, analog_state.length, buf_builder, states);
          break;
      }
    }
  }

  if (config.num_reg_cmd.has_value())
  {
    for (const gpio_config::NumRegConfig& num_reg_cmd : *config.num_reg_cmd)
    {
      const uint32_t start = num_reg_cmd.start;
      const uint32_t length = num_reg_cmd.length;
      auto& block = buf_builder.addCommandConfig<CommandGPIOTypes::FloatReg, float>(start, length);
      for (uint32_t index = start; index < start + length; ++index)
      {
        commands.emplace_back(std::make_unique<IOCommand<CommandGPIOTypes::FloatReg, float>>(index, block));
      }
    }
  }

  if (config.num_reg_state.has_value())
  {
    for (const gpio_config::NumRegConfig& num_reg_state : *config.num_reg_state)
    {
      const uint32_t start = num_reg_state.start;
      const uint32_t length = num_reg_state.length;
      auto& block = buf_builder.addStatusConfig<StatusGPIOTypes::FloatReg, float>(start, length);
      for (uint32_t index = start; index < start + length; ++index)
      {
        states.emplace_back(std::make_unique<IOState<StatusGPIOTypes::FloatReg, float>>(index, block));
      }
    }
  }

  return { std::move(commands), std::move(states), buf_builder.build() };
}

}  // namespace

FanucHardwareInterface::FanucHardwareInterface()
  : fr_joint_pos_{ Eigen::VectorXd::Zero(9) }
  , fr_prev_joint_pos_{ Eigen::VectorXd::Zero(9) }
  , fr_joint_vel_{ Eigen::VectorXd::Zero(9) }
  , joint_targets_{ Eigen::VectorXd::Zero(9) }
  , joint_targets_degrees_{ Eigen::VectorXd::Zero(9) }
  , stream_motion_port_(60015)
  , rmi_port_(1600)
{
}

FanucHardwareInterface::~FanucHardwareInterface() = default;

hardware_interface::CallbackReturn FanucHardwareInterface::on_init(const hardware_interface::HardwareInfo& info)
{
  if (SystemInterface::on_init(info) != CallbackReturn::SUCCESS)
  {
    return CallbackReturn::ERROR;
  }
  info_ = info;

  // Parse and configure cyclic GPIO from the yaml config.
  const auto config_path_it = info_.hardware_parameters.find("gpio_configuration");
  if (config_path_it != info_.hardware_parameters.end() && !config_path_it->second.empty())
  {
    gpio_config::GPIOConfig gpio_config;
    RCLCPP_INFO_STREAM(rclcpp::get_logger(kFRHWInterface),
                       "Loading GPIO configuration file: " << config_path_it->second);
    try
    {
      gpio_config = gpio_config::ParseGPIOConfig(std::filesystem::path(config_path_it->second));
    }
    catch (std::runtime_error& e)
    {
      RCLCPP_ERROR(rclcpp::get_logger(kFRHWInterface), "Failed to parse gpio_config: %s", e.what());
      return CallbackReturn::ERROR;
    }

    IOInterfaces interfaces = GPIOConfigToInterfaces(gpio_config.gpio_topic_config);
    io_commands_ = std::move(interfaces.commands);
    io_state_ = std::move(interfaces.states);
    gpio_buffer_ = std::make_shared<fanuc_client::GPIOBuffer>(std::move(interfaces.buffer));
  }

  io_state_names_.reserve(io_state_.size());
  for (const auto& io_state : io_state_)
  {
    io_state_names_.push_back(io_state->name() + "/" + std::to_string(io_state->index));
  }
  io_command_names_.reserve(io_commands_.size());
  for (const auto& io_command : io_commands_)
  {
    io_command_names_.push_back(io_command->name() + "/" + std::to_string(io_command->index));
  }

  joint_position_state_names_.resize(info_.joints.size());
  joint_velocity_state_names_.resize(info_.joints.size());
  joint_position_command_names_.resize(info_.joints.size());
  for (size_t i = 0; i < info_.joints.size(); ++i)
  {
    joint_position_state_names_[i] = info_.joints[i].name + "/" + hardware_interface::HW_IF_POSITION;
    joint_velocity_state_names_[i] = info_.joints[i].name + "/" + hardware_interface::HW_IF_VELOCITY;
    joint_position_command_names_[i] = info_.joints[i].name + "/" + hardware_interface::HW_IF_POSITION;
  }

  // is_connected is pushed separately at the top of read(), since it must be pushed even on the
  // early-return-on-disconnected path this list is only used from the try block below.
  fixed_state_interfaces_ = {
    { std::string(kRobotStatusInterfaceName) + "/" + kStatusInErrorType, &robot_status_.in_error },
    { std::string(kRobotStatusInterfaceName) + "/" + kStatusTPEnabledType, &robot_status_.tp_enabled },
    { std::string(kRobotStatusInterfaceName) + "/" + kStatusEStoppedType, &robot_status_.e_stopped },
    { std::string(kRobotStatusInterfaceName) + "/" + kStatusMotionPossibleType, &robot_status_.motion_possible },
    { std::string(kRobotStatusInterfaceName) + "/" + kStatusContactStopModeType,
      &robot_status_.contact_stop_mode },
    { std::string(kRobotStatusInterfaceName) + "/" + kStatusCollaborativeSpeedScalingType,
      &robot_status_.collaborative_speed_scaling },
    { std::string(kForceInterfaceName) + "/" + kForceXType, &force_sensor_.force_x },
    { std::string(kForceInterfaceName) + "/" + kForceYType, &force_sensor_.force_y },
    { std::string(kForceInterfaceName) + "/" + kForceZType, &force_sensor_.force_z },
    { std::string(kForceInterfaceName) + "/" + kMomentXType, &force_sensor_.moment_x },
    { std::string(kForceInterfaceName) + "/" + kMomentYType, &force_sensor_.moment_y },
    { std::string(kForceInterfaceName) + "/" + kMomentZType, &force_sensor_.moment_z },
    { std::string(kForceInterfaceName) + "/" + kForceSensorType, &force_sensor_.fs_type },
    // For force_torque_sensor_broadcaster (geometry_msgs/WrenchStamped)
    { "ft_sensor/force.x", &force_sensor_.force_x },
    { "ft_sensor/force.y", &force_sensor_.force_y },
    { "ft_sensor/force.z", &force_sensor_.force_z },
    { "ft_sensor/torque.x", &force_sensor_.moment_x },
    { "ft_sensor/torque.y", &force_sensor_.moment_y },
    { "ft_sensor/torque.z", &force_sensor_.moment_z },
  };

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn
FanucHardwareInterface::on_configure(const rclcpp_lifecycle::State& /*previous_state*/)
{
  RCLCPP_INFO_STREAM(rclcpp::get_logger(kFRHWInterface), "Preparing FANUC ROS2 HW interface");
  ip_address_ = info_.hardware_parameters["robot_ip"];
  try
  {
    rmi_port_ = StringToInt("rmi_port", info_.hardware_parameters["rmi_port"]);
    stream_motion_port_ = StringToInt("stream_motion_port", info_.hardware_parameters["stream_motion_port"]);
    payload_schedule_ = StringToInt("payload_schedule", info_.hardware_parameters["payload_schedule"]);
    out_cmd_interp_buff_target_ =
        StringToInt("out_cmd_interp_buff_target", info_.hardware_parameters["out_cmd_interp_buff_target"]);
    force_sensor_type_ = StringToInt("force_sensor_type", info_.hardware_parameters["force_sensor_type"]);
  }
  catch (const std::exception& e)
  {
    return CallbackReturn::ERROR;
  }

  RCLCPP_INFO_STREAM(rclcpp::get_logger(kFRHWInterface), "payload_schedule: " << payload_schedule_);
  RCLCPP_INFO_STREAM(rclcpp::get_logger(kFRHWInterface), "Starting RMI with: " << ip_address_);

  // Initialize the driver client
  for (int i = 0; i < kNumberConnectionAttempts; i++)
  {
    RCLCPP_INFO_STREAM(rclcpp::get_logger(kFRHWInterface), "Connecting to the robot: attempt: " << i);
    try
    {
      fanuc_client_.reset();
      fanuc_client_ = std::make_unique<fanuc_client::FanucClient>(ip_address_, stream_motion_port_, rmi_port_);
      fanuc_client_->setOutCmdInterpBuffTarget(out_cmd_interp_buff_target_);
      fanuc_client_->setForceSensorType(force_sensor_type_);
      fanuc_client_->startRMI();
      fanuc_client_->setPayloadSchedule(payload_schedule_);
      fanuc_client_->validateGPIOBuffer(gpio_buffer_);
      RCLCPP_INFO_STREAM(rclcpp::get_logger(kFRHWInterface), "Successfully connected to the robot.");
      RCLCPP_INFO_STREAM(rclcpp::get_logger(kFRHWInterface),
                         "FANUC ROS2 HW interface is ready with client version: " << fanuc_client_->getClientVersion());
      return CallbackReturn::SUCCESS;
    }
    catch (std::runtime_error& e)
    {
      RCLCPP_WARN(rclcpp::get_logger(kFRHWInterface), "%s", e.what());
      rclcpp::sleep_for(std::chrono::milliseconds(3000));
    }
  }

  RCLCPP_ERROR(rclcpp::get_logger(kFRHWInterface), "Failed to connect to the robot.");
  return CallbackReturn::ERROR;
}

hardware_interface::CallbackReturn FanucHardwareInterface::on_activate(const rclcpp_lifecycle::State& previous_state)
{
  RCLCPP_INFO_STREAM(rclcpp::get_logger(kFRHWInterface), "activating hardware interface");

  fanuc_client_->startRealtimeStream(gpio_buffer_);
  joint_targets_degrees_ = fanuc_client_->readJointAngles();
  joint_targets_.array() = M_PI / 180.0 * joint_targets_degrees_.array();

  return CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn FanucHardwareInterface::on_deactivate(const rclcpp_lifecycle::State& previous_state)
{
  RCLCPP_INFO_STREAM(rclcpp::get_logger(kFRHWInterface), "deactivating stream motion");
  fanuc_client_->stopRealtimeStream();
  return CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn FanucHardwareInterface::on_cleanup(const rclcpp_lifecycle::State& previous_state)
{
  fanuc_client_.reset();
  return CallbackReturn::SUCCESS;
}

namespace
{
hardware_interface::InterfaceDescription MakeUnlistedInterface(const std::string& prefix, const std::string& name)
{
  hardware_interface::InterfaceInfo info{};
  info.name = name;
  info.initial_value = "0";
  return hardware_interface::InterfaceDescription(prefix, info);
}
}  // namespace

std::vector<hardware_interface::InterfaceDescription>
FanucHardwareInterface::export_unlisted_state_interface_descriptions()
{
  std::vector<hardware_interface::InterfaceDescription> descriptions;

  for (const auto& io_state : io_state_)
  {
    descriptions.push_back(MakeUnlistedInterface(io_state->name(), std::to_string(io_state->index)));
  }

  descriptions.push_back(MakeUnlistedInterface(kRobotStatusInterfaceName, kStatusInErrorType));
  descriptions.push_back(MakeUnlistedInterface(kRobotStatusInterfaceName, kStatusTPEnabledType));
  descriptions.push_back(MakeUnlistedInterface(kRobotStatusInterfaceName, kStatusEStoppedType));
  descriptions.push_back(MakeUnlistedInterface(kRobotStatusInterfaceName, kStatusMotionPossibleType));
  descriptions.push_back(MakeUnlistedInterface(kRobotStatusInterfaceName, kStatusContactStopModeType));
  descriptions.push_back(
      MakeUnlistedInterface(kRobotStatusInterfaceName, kStatusCollaborativeSpeedScalingType));

  descriptions.push_back(MakeUnlistedInterface(kConnectionStatusName, kIsConnectedType));

  descriptions.push_back(MakeUnlistedInterface(kForceInterfaceName, kForceXType));
  descriptions.push_back(MakeUnlistedInterface(kForceInterfaceName, kForceYType));
  descriptions.push_back(MakeUnlistedInterface(kForceInterfaceName, kForceZType));
  descriptions.push_back(MakeUnlistedInterface(kForceInterfaceName, kMomentXType));
  descriptions.push_back(MakeUnlistedInterface(kForceInterfaceName, kMomentYType));
  descriptions.push_back(MakeUnlistedInterface(kForceInterfaceName, kMomentZType));
  descriptions.push_back(MakeUnlistedInterface(kForceInterfaceName, kForceSensorType));

  // For force_torque_sensor_broadcaster (geometry_msgs/WrenchStamped)
  descriptions.push_back(MakeUnlistedInterface("ft_sensor", "force.x"));
  descriptions.push_back(MakeUnlistedInterface("ft_sensor", "force.y"));
  descriptions.push_back(MakeUnlistedInterface("ft_sensor", "force.z"));
  descriptions.push_back(MakeUnlistedInterface("ft_sensor", "torque.x"));
  descriptions.push_back(MakeUnlistedInterface("ft_sensor", "torque.y"));
  descriptions.push_back(MakeUnlistedInterface("ft_sensor", "torque.z"));

  return descriptions;
}

std::vector<hardware_interface::InterfaceDescription>
FanucHardwareInterface::export_unlisted_command_interface_descriptions()
{
  std::vector<hardware_interface::InterfaceDescription> descriptions;
  for (const auto& io_command : io_commands_)
  {
    descriptions.push_back(MakeUnlistedInterface(io_command->name(), std::to_string(io_command->index)));
  }
  return descriptions;
}

hardware_interface::return_type FanucHardwareInterface::read(const rclcpp::Time& /*time*/,
                                                             const rclcpp::Duration& period)
{
  robot_status_.is_connected = fanuc_client_ != nullptr && fanuc_client_->isStreaming();
  set_state(std::string(kConnectionStatusName) + "/" + kIsConnectedType, robot_status_.is_connected);
  if (!robot_status_.is_connected)
  {
    if (fanuc_client_ != nullptr)
    {
      try
      {
        fanuc_client_->stopRealtimeStream();
      }
      catch (const std::runtime_error& e)
      {
        RCLCPP_DEBUG(rclcpp::get_logger(kFRHWInterface), "Stream already stopped: %s", e.what());
      }
      catch (...)
      {
        // Catch any other exceptions during shutdown
        RCLCPP_DEBUG(rclcpp::get_logger(kFRHWInterface), "Exception during stream shutdown (likely normal)");
      }
    }

    static auto last_log_time = std::chrono::steady_clock::now();
    auto now = std::chrono::steady_clock::now();
    if (std::chrono::duration_cast<std::chrono::milliseconds>(now - last_log_time).count() > 5000)
    {
      RCLCPP_WARN(rclcpp::get_logger(kFRHWInterface),
                  "FANUC ROS2 HW no longer streaming (this is normal during shutdown).");
      last_log_time = now;
    }
    return hardware_interface::return_type::ERROR;
  }

  try
  {
    fr_prev_joint_pos_ = fr_joint_pos_;
    const Eigen::Ref<const Eigen::VectorXd> joint_angles = fanuc_client_->readJointAngles();
    for (Eigen::Index i = 0; i < joint_angles.size(); ++i)
    {
      fr_joint_pos_[i] = M_PI / 180.0 * joint_angles[i];
    }
    if ((fr_prev_joint_pos_.array() != fr_joint_pos_.array()).any())
    {
      const double dt = static_cast<double>(fanuc_client_->getControlPeriod()) / 1000.0;
      fr_joint_vel_ = (fr_joint_pos_ - fr_prev_joint_pos_) / dt;
    }
    for (size_t i = 0; i < joint_position_state_names_.size(); ++i)
    {
      set_state(joint_position_state_names_[i], fr_joint_pos_[i]);
      set_state(joint_velocity_state_names_[i], fr_joint_vel_[i]);
    }

    for (size_t i = 0; i < io_state_.size(); ++i)
    {
      io_state_[i]->updateValue();
      set_state(io_state_names_[i], io_state_[i]->value);
    }

    robot_status_.in_error = fanuc_client_->robot_status().in_error;
    robot_status_.tp_enabled = fanuc_client_->robot_status().tp_enabled;
    robot_status_.e_stopped = fanuc_client_->robot_status().e_stopped;
    robot_status_.motion_possible = fanuc_client_->robot_status().motion_possible;
    robot_status_.contact_stop_mode = static_cast<double>(fanuc_client_->robot_status().contact_stop_mode);
    robot_status_.collaborative_speed_scaling = static_cast<double>(fanuc_client_->robot_status().safety_scale);

    force_sensor_.force_x = static_cast<double>(fanuc_client_->force_sensor().force_x);
    force_sensor_.force_y = static_cast<double>(fanuc_client_->force_sensor().force_y);
    force_sensor_.force_z = static_cast<double>(fanuc_client_->force_sensor().force_z);
    force_sensor_.moment_x = static_cast<double>(fanuc_client_->force_sensor().moment_x);
    force_sensor_.moment_y = static_cast<double>(fanuc_client_->force_sensor().moment_y);
    force_sensor_.moment_z = static_cast<double>(fanuc_client_->force_sensor().moment_z);
    force_sensor_.fs_type = static_cast<double>(fanuc_client_->force_sensor().fs_type);

    for (const auto& [name, value_ptr] : fixed_state_interfaces_)
    {
      set_state(name, *value_ptr);
    }
  }
  catch (const std::exception& e)
  {
    RCLCPP_DEBUG(rclcpp::get_logger(kFRHWInterface), "Exception during read (likely shutdown): %s", e.what());
    return hardware_interface::return_type::ERROR;
  }
  catch (...)
  {
    RCLCPP_DEBUG(rclcpp::get_logger(kFRHWInterface), "Unknown exception during read (likely shutdown)");
    return hardware_interface::return_type::ERROR;
  }

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type FanucHardwareInterface::write(const rclcpp::Time& time, const rclcpp::Duration& period)
{
  for (size_t i = 0; i < joint_position_command_names_.size(); ++i)
  {
    joint_targets_[i] = get_command<double>(joint_position_command_names_[i]);
  }
  for (size_t i = 0; i < io_commands_.size(); ++i)
  {
    io_commands_[i]->value = get_command<double>(io_command_names_[i]);
  }

  robot_status_.is_connected = fanuc_client_ != nullptr && fanuc_client_->isStreaming();
  if (!robot_status_.is_connected)
  {
    if (fanuc_client_ != nullptr)
    {
      try
      {
        fanuc_client_->stopRealtimeStream();
      }
      catch (const std::runtime_error& e)
      {
        RCLCPP_DEBUG(rclcpp::get_logger(kFRHWInterface), "Stream already stopped: %s", e.what());
      }
      catch (...)
      {
        RCLCPP_DEBUG(rclcpp::get_logger(kFRHWInterface), "Exception during stream shutdown (likely normal)");
      }
    }
    // Throttle to avoid spam during shutdown
    static auto last_log_time = std::chrono::steady_clock::now();
    auto now = std::chrono::steady_clock::now();
    if (std::chrono::duration_cast<std::chrono::milliseconds>(now - last_log_time).count() > 5000)
    {
      RCLCPP_WARN(rclcpp::get_logger(kFRHWInterface),
                  "FANUC ROS2 HW no longer streaming (this is normal during shutdown).");
      last_log_time = now;
    }
    return hardware_interface::return_type::ERROR;
  }

  try
  {
    joint_targets_degrees_.array() = 180.0 / M_PI * joint_targets_.array();
    fanuc_client_->writeJointTarget(joint_targets_degrees_);

    for (const auto& io_command : io_commands_)
    {
      io_command->updateBuffer();
    }
    fanuc_client_->sendIOCommand();
  }
  catch (const std::exception& e)
  {
    // During shutdown, operations may fail - log but don't crash
    RCLCPP_DEBUG(rclcpp::get_logger(kFRHWInterface), "Exception during write (likely shutdown): %s", e.what());
    return hardware_interface::return_type::ERROR;
  }
  catch (...)
  {
    // Catch any other exceptions during shutdown
    RCLCPP_DEBUG(rclcpp::get_logger(kFRHWInterface), "Unknown exception during write (likely shutdown)");
    return hardware_interface::return_type::ERROR;
  }

  return hardware_interface::return_type::OK;
}

hardware_interface::CallbackReturn FanucHardwareInterface::on_shutdown(const rclcpp_lifecycle::State& previous_state)
{
  return hardware_interface::CallbackReturn::SUCCESS;
}
}  // namespace fanuc_robot_driver

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(fanuc_robot_driver::FanucHardwareInterface, hardware_interface::SystemInterface)
