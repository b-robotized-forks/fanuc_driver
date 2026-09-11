// SPDX-FileCopyrightText: 2025, FANUC America Corporation
// SPDX-FileCopyrightText: 2025, FANUC CORPORATION
//
// SPDX-License-Identifier: Apache-2.0

#pragma once

#include <utility>

#include "fanuc_client/fanuc_client.hpp"
#include "fanuc_client/gpio_buffer.hpp"
#include "gpio_config/gpio_config.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp_lifecycle/state.hpp"

namespace fanuc_robot_driver
{

struct IOCommandInterface;
struct IOStateInterface;

class FanucHardwareInterface : public hardware_interface::SystemInterface
{
public:
  FanucHardwareInterface();

  FanucHardwareInterface(const FanucHardwareInterface&) = delete;

  FanucHardwareInterface& operator=(const FanucHardwareInterface&) = delete;

  ~FanucHardwareInterface() override;

  hardware_interface::CallbackReturn on_init(const hardware_interface::HardwareInfo& info) override;

  hardware_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State& previous_state) override;

  hardware_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State& previous_state) override;

  hardware_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State& previous_state) override;

  hardware_interface::CallbackReturn on_cleanup(const rclcpp_lifecycle::State& previous_state) override;

  hardware_interface::CallbackReturn on_shutdown(const rclcpp_lifecycle::State& previous_state) override;

  // The joint position/velocity state and position command interfaces are declared in the URDF
  // and already picked up by the default on_export_state_interfaces()/
  // on_export_command_interfaces(). All GPIO, status and force-sensor interfaces are not tied to
  // a URDF joint (the GPIO ones come from a runtime-parsed config file) and need explicit
  // declaration here.
  std::vector<hardware_interface::InterfaceDescription> export_unlisted_state_interface_descriptions()
      final;

  std::vector<hardware_interface::InterfaceDescription>
  export_unlisted_command_interface_descriptions() final;

  hardware_interface::return_type read(const rclcpp::Time& time, const rclcpp::Duration& period) override;

  hardware_interface::return_type write(const rclcpp::Time& time, const rclcpp::Duration& period) override;

private:
  struct RobotStatusValues
  {
    // Unless otherwise stated, all values are bools.
    double in_error{};
    double tp_enabled{};
    double e_stopped{};
    double motion_possible{};
    // Enum type:
    // None = 0
    // SAFE = 1
    // STOP = 2
    // DSBL = 3
    // ESCP = 4
    double contact_stop_mode{};
    // unsigned int
    double collaborative_speed_scaling{};

    double is_connected{};
  };

  struct ForceSensorValues
  {
    // float
    double force_x{};
    double force_y{};
    double force_z{};
    double moment_x{};
    double moment_y{};
    double moment_z{};
    // uint32_t
    double fs_type{};
  };

  std::unique_ptr<fanuc_client::FanucClient> fanuc_client_;
  Eigen::VectorXd fr_joint_pos_;
  Eigen::VectorXd fr_prev_joint_pos_;
  Eigen::VectorXd fr_joint_vel_;
  Eigen::VectorXd joint_targets_;
  Eigen::VectorXd joint_targets_degrees_;
  RobotStatusValues robot_status_;
  ForceSensorValues force_sensor_;
  std::string ip_address_;
  int32_t payload_schedule_;
  uint16_t stream_motion_port_;
  uint16_t rmi_port_;
  uint32_t out_cmd_interp_buff_target_;
  uint32_t force_sensor_type_;

  std::shared_ptr<fanuc_client::GPIOBuffer> gpio_buffer_;

  std::vector<std::unique_ptr<IOCommandInterface>> io_commands_;
  std::vector<std::unique_ptr<IOStateInterface>> io_state_;

  // Interface names, built once in on_init() (matching kassow_kord_hardware_interface's
  // convention) instead of concatenating "<prefix>/<interface>" fresh on every read()/write()
  // cycle. set_state()/get_command() still do a name lookup per call - only the string-building
  // is cached, not the resolved handle.
  std::vector<std::string> joint_position_state_names_;
  std::vector<std::string> joint_velocity_state_names_;
  std::vector<std::string> joint_position_command_names_;
  std::vector<std::string> io_state_names_;
  std::vector<std::string> io_command_names_;
  // Fixed-name state interfaces (robot status, connection status, force sensor) that are not
  // tied to a joint or the GPIO config: cached name plus a pointer to the source value, so
  // read() can push all of them in a single loop.
  std::vector<std::pair<std::string, double*>> fixed_state_interfaces_;
};

}  // namespace fanuc_robot_driver
