// SPDX-FileCopyrightText: 2025-2026, FANUC America Corporation
// SPDX-FileCopyrightText: 2025-2026, FANUC CORPORATION
//
// SPDX-License-Identifier: Apache-2.0

#include "fanuc_controllers/scaled_joint_trajectory_controller.hpp"

#include <algorithm>
#include <cmath>
#include <type_traits>
#include <utility>

#include "fanuc_robot_driver/constants.hpp"

namespace
{

const std::string kMotionPossibleInterfaceName =  // NOLINT(runtime/string)
    std::string(fanuc_robot_driver::kRobotStatusInterfaceName) + "/" + fanuc_robot_driver::kStatusMotionPossibleType;
const std::string kRobotScalingInterfaceName =  // NOLINT(runtime/string)
    std::string(fanuc_robot_driver::kRobotStatusInterfaceName) + "/" +
    fanuc_robot_driver::kStatusCollaborativeSpeedScalingType;
constexpr auto kMotionNotPossibleError = "Aborted: STMO is inactive (!motion_possible)";

// The active goal is stored in a realtime_tools::RealtimeBuffer on Jazzy
// and in a realtime_tools::RealtimeThreadSafeBox on newer distributions.
template <typename T, typename = void>
struct IsRealtimeBuffer : std::false_type
{
};

template <typename T>
struct IsRealtimeBuffer<T, std::void_t<decltype(std::declval<T&>().readFromRT())>> : std::true_type
{
};

// Returns the active goal and removes it from the buffer. Returns nullptr if there is no active goal
// or if the buffer is locked.
template <typename GoalHandlePtr, typename GoalBuffer>
GoalHandlePtr take_active_goal(GoalBuffer& goal_buffer)
{
  GoalHandlePtr goal;
  if constexpr (IsRealtimeBuffer<GoalBuffer>::value)
  {
    goal = *goal_buffer.readFromRT();
    if (goal)
    {
      goal_buffer.writeFromNonRT(GoalHandlePtr());
    }
  }
  else
  {
    goal_buffer.try_get([&](const GoalHandlePtr& active_goal) { goal = active_goal; });
    if (goal && !goal_buffer.try_set([](GoalHandlePtr& active_goal) { active_goal = GoalHandlePtr(); }))
    {
      goal.reset();
    }
  }
  return goal;
}

}  // namespace

namespace fanuc_controllers
{

controller_interface::InterfaceConfiguration ScaledJointTrajectoryController::state_interface_configuration() const
{
  auto state_interface_config = JointTrajectoryController::state_interface_configuration();
  state_interface_config.names.push_back(kRobotScalingInterfaceName);
  state_interface_config.names.push_back(kMotionPossibleInterfaceName);
  return state_interface_config;
}

controller_interface::CallbackReturn
ScaledJointTrajectoryController::on_configure(const rclcpp_lifecycle::State& previous_state)
{
  const auto ret = JointTrajectoryController::on_configure(previous_state);
  if (ret != controller_interface::CallbackReturn::SUCCESS)
  {
    return ret;
  }

  if (!params_.speed_scaling.state_interface.empty())
  {
    RCLCPP_WARN(get_node()->get_logger(),
                "Parameter 'speed_scaling.state_interface' is ignored. This controller reads the speed scaling from "
                "'%s' and the topic 'speed_scaling_factor'.",
                kRobotScalingInterfaceName.c_str());
  }

  time_scale_subscriber_ = get_node()->create_subscription<std_msgs::msg::Int32>(
      "speed_scaling_factor", 10,
      std::bind(&ScaledJointTrajectoryController::time_scale_callback, this, std::placeholders::_1));

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn
ScaledJointTrajectoryController::on_activate(const rclcpp_lifecycle::State& previous_state)
{
  const auto ret = JointTrajectoryController::on_activate(previous_state);
  if (ret != controller_interface::CallbackReturn::SUCCESS)
  {
    return ret;
  }

  for (auto& interface : state_interfaces_)
  {
    if (interface.get_name() == kMotionPossibleInterfaceName)
    {
      motion_possible_interface_ = interface;
    }
    else if (interface.get_name() == kRobotScalingInterfaceName)
    {
      robot_scaling_interface_ = interface;
    }
  }
  if (!motion_possible_interface_ || !robot_scaling_interface_)
  {
    RCLCPP_ERROR(get_node()->get_logger(), "Missing state interface '%s' or '%s'.",
                 kMotionPossibleInterfaceName.c_str(), kRobotScalingInterfaceName.c_str());
    return controller_interface::CallbackReturn::ERROR;
  }

  last_motion_possible_ = false;
  robot_scaling_factor_ = 1.0;
  folag_state_ = time_scale_value_.load();
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn
ScaledJointTrajectoryController::on_deactivate(const rclcpp_lifecycle::State& previous_state)
{
  motion_possible_interface_.reset();
  robot_scaling_interface_.reset();
  return JointTrajectoryController::on_deactivate(previous_state);
}

controller_interface::return_type ScaledJointTrajectoryController::update(const rclcpp::Time& time,
                                                                          const rclcpp::Duration& period)
{
  if (motion_possible_interface_->get().get_optional().value_or(0.0) == 0.0)
  {
    // STMO does not have the motion control: do not command the robot.
    last_motion_possible_ = false;
    abort_active_goal(kMotionNotPossibleError);
    return controller_interface::return_type::OK;
  }

  if (!last_motion_possible_)
  {
    // Motion is possible again: hold the current position.
    read_state_from_state_interfaces(state_current_);
    abort_active_goal(kMotionNotPossibleError);
    rt_has_pending_goal_ = false;
    add_new_trajectory_msg(set_hold_position());
    last_motion_possible_ = true;
  }

  const double robot_scaling = robot_scaling_interface_->get().get_optional().value_or(robot_scaling_factor_);
  if (std::isfinite(robot_scaling))
  {
    robot_scaling_factor_ = robot_scaling;
  }
  const double time_scale = 0.01 * first_order_lag_filter(time_scale_value_.load(), period.seconds());
  scaling_factor_.store(robot_scaling_factor_ * time_scale);

  return JointTrajectoryController::update(time, period);
}

void ScaledJointTrajectoryController::abort_active_goal(const std::string& error_string)
{
  const auto active_goal = take_active_goal<RealtimeGoalHandlePtr>(rt_active_goal_);
  if (!active_goal)
  {
    return;
  }
  auto result = std::make_shared<FollowJTrajAction::Result>();
  result->set__error_code(FollowJTrajAction::Result::INVALID_GOAL);
  result->set__error_string(error_string);
  active_goal->setAborted(result);
  rt_has_pending_goal_ = false;
  RCLCPP_WARN(get_node()->get_logger(), "%s", error_string.c_str());
}

double ScaledJointTrajectoryController::first_order_lag_filter(const double filter_input, const double period)
{
  // Discrete time first order lag filter
  const double alpha = std::clamp(period / folag_tau_, 0.0, 1.0);
  folag_state_ = (1.0 - alpha) * folag_state_ + alpha * filter_input;
  return folag_state_;
}

void ScaledJointTrajectoryController::time_scale_callback(const std::shared_ptr<std_msgs::msg::Int32> msg)
{
  // Clamp the topic value between 0 and 100 %
  time_scale_value_.store(std::clamp(msg->data, 0, 100));
}

}  // namespace fanuc_controllers

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(fanuc_controllers::ScaledJointTrajectoryController, controller_interface::ControllerInterface)
