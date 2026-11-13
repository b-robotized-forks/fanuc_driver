// SPDX-FileCopyrightText: 2025-2026, FANUC America Corporation
// SPDX-FileCopyrightText: 2025-2026, FANUC CORPORATION
//
// SPDX-License-Identifier: Apache-2.0

#ifndef FANUC_CONTROLLERS__SCALED_JOINT_TRAJECTORY_CONTROLLER_HPP_
#define FANUC_CONTROLLERS__SCALED_JOINT_TRAJECTORY_CONTROLLER_HPP_

#include <atomic>
#include <functional>
#include <memory>
#include <optional>
#include <string>

#include "joint_trajectory_controller/joint_trajectory_controller.hpp"
#include "rclcpp/subscription.hpp"
#include "std_msgs/msg/int32.hpp"

namespace fanuc_controllers
{

/**
 * Joint trajectory controller with the FANUC speed scaling and motion state.
 *
 * The speed scaling factor of the base controller is the product of the robot safety scale
 * (state interface robot_status/collaborative_speed_scaling, 0 to 1) and the filtered value of
 * the topic speed_scaling_factor (std_msgs/Int32, 0 to 100 %).
 *
 * While robot_status/motion_possible is false, the controller aborts the active goal and does not
 * write commands. When motion is possible again, it holds the current position.
 */
class ScaledJointTrajectoryController : public joint_trajectory_controller::JointTrajectoryController
{
public:
  ScaledJointTrajectoryController() = default;
  ~ScaledJointTrajectoryController() override = default;

  controller_interface::InterfaceConfiguration state_interface_configuration() const override;

  controller_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State& previous_state) override;

  controller_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State& previous_state) override;

  controller_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State& previous_state) override;

  controller_interface::return_type update(const rclcpp::Time& time, const rclcpp::Duration& period) override;

private:
  using StateInterfaceRef = std::reference_wrapper<hardware_interface::LoanedStateInterface>;

  void time_scale_callback(const std::shared_ptr<std_msgs::msg::Int32> msg);
  double first_order_lag_filter(const double filter_input, const double period);
  void abort_active_goal(const std::string& error_string);

  std::optional<StateInterfaceRef> motion_possible_interface_;
  std::optional<StateInterfaceRef> robot_scaling_interface_;
  bool last_motion_possible_ = false;
  double robot_scaling_factor_ = 1.0;

  rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr time_scale_subscriber_ = nullptr;
  std::atomic<int> time_scale_value_{ 100 };  // Topic value in %, default 100 % (no scaling)
  double folag_state_ = 100.0;
  double folag_tau_ = 0.2;  // Time constant of the first order lag filter in seconds
};

}  // namespace fanuc_controllers

#endif  // FANUC_CONTROLLERS__SCALED_JOINT_TRAJECTORY_CONTROLLER_HPP_
