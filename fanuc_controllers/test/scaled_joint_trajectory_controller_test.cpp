// SPDX-FileCopyrightText: 2026, FANUC America Corporation
// SPDX-FileCopyrightText: 2026, FANUC CORPORATION
//
// SPDX-License-Identifier: Apache-2.0

#include <gtest/gtest.h>

#include <chrono>
#include <future>
#include <limits>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include "control_msgs/action/follow_joint_trajectory.hpp"
#include "fanuc_controllers/scaled_joint_trajectory_controller.hpp"
#include "fanuc_robot_driver/constants.hpp"
#include "lifecycle_msgs/msg/state.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "std_msgs/msg/int32.hpp"

using namespace std::chrono_literals;
using FollowJTrajAction = control_msgs::action::FollowJointTrajectory;

namespace
{

constexpr auto kRobotDescription = R"(<?xml version="1.0"?>
<robot name="test_robot">
  <link name="base_link"/>
  <link name="link1"/>
  <joint name="joint1" type="revolute">
    <parent link="base_link"/>
    <child link="link1"/>
    <axis xyz="0 0 1"/>
    <limit lower="-3.0" upper="3.0" effort="10.0" velocity="10.0"/>
  </joint>
</robot>
)";

class TestableScaledJointTrajectoryController : public fanuc_controllers::ScaledJointTrajectoryController
{
public:
  double get_scaling_factor() const
  {
    return scaling_factor_.load();
  }
};

class ScaledJointTrajectoryControllerTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    controller_ = std::make_shared<TestableScaledJointTrajectoryController>();

    controller_interface::ControllerInterfaceParams params;
    params.controller_name = "test_scaled_jtc";
    params.update_rate = 100;
    params.controller_manager_update_rate = 100;
    params.robot_description = kRobotDescription;
    params.node_options.parameter_overrides({ { "joints", std::vector<std::string>{ "joint1" } },
                                              { "command_interfaces", std::vector<std::string>{ "position" } },
                                              { "state_interfaces", std::vector<std::string>{ "position" } } });
    ASSERT_EQ(controller_->init(params), controller_interface::return_type::OK);
    ASSERT_EQ(controller_->configure().id(), lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE);

    position_command_ = make_handle<hardware_interface::CommandInterface>("joint1", "position", 0.0);
    position_state_ = make_handle<hardware_interface::StateInterface>("joint1", "position", 0.0);
    robot_scaling_state_ = make_handle<hardware_interface::StateInterface>(
        fanuc_robot_driver::kRobotStatusInterfaceName, fanuc_robot_driver::kStatusCollaborativeSpeedScalingType, 1.0);
    motion_possible_state_ = make_handle<hardware_interface::StateInterface>(
        fanuc_robot_driver::kRobotStatusInterfaceName, fanuc_robot_driver::kStatusMotionPossibleType, 1.0);

    std::vector<hardware_interface::LoanedCommandInterface> command_interfaces;
    command_interfaces.emplace_back(position_command_, nullptr);
    std::vector<hardware_interface::LoanedStateInterface> state_interfaces;
    state_interfaces.emplace_back(position_state_, nullptr);
    state_interfaces.emplace_back(robot_scaling_state_, nullptr);
    state_interfaces.emplace_back(motion_possible_state_, nullptr);
    controller_->assign_interfaces(std::move(command_interfaces), std::move(state_interfaces));

    ASSERT_EQ(controller_->get_node()->activate().id(), lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE);

    client_node_ = std::make_shared<rclcpp::Node>("test_client");
    executor_.add_node(controller_->get_node()->get_node_base_interface());
    executor_.add_node(client_node_);
    executor_thread_ = std::thread([this]() { executor_.spin(); });

    time_ = controller_->get_node()->now();
  }

  void TearDown() override
  {
    executor_.cancel();
    executor_thread_.join();
    controller_->get_node()->deactivate();
  }

  template <typename HandleT>
  static std::shared_ptr<HandleT> make_handle(const std::string& prefix, const std::string& name, double value)
  {
    hardware_interface::InterfaceInfo info;
    info.name = name;
    info.initial_value = std::to_string(value);
    return std::make_shared<HandleT>(hardware_interface::InterfaceDescription(prefix, info));
  }

  template <typename HandleT>
  static void set(const std::shared_ptr<HandleT>& handle, double value)
  {
    ASSERT_TRUE(handle->set_value(value));
  }

  template <typename HandleT>
  static double get(const std::shared_ptr<HandleT>& handle)
  {
    return handle->template get_optional<double>().value_or(std::numeric_limits<double>::quiet_NaN());
  }

  void update(size_t cycles = 1)
  {
    const rclcpp::Duration period = rclcpp::Duration::from_seconds(0.01);
    for (size_t i = 0; i < cycles; ++i)
    {
      time_ += period;
      ASSERT_EQ(controller_->update(time_, period), controller_interface::return_type::OK);
      // The robot follows the command.
      set(position_state_, get(position_command_));
    }
  }

  std::shared_ptr<TestableScaledJointTrajectoryController> controller_;
  std::shared_ptr<hardware_interface::CommandInterface> position_command_;
  std::shared_ptr<hardware_interface::StateInterface> position_state_;
  std::shared_ptr<hardware_interface::StateInterface> robot_scaling_state_;
  std::shared_ptr<hardware_interface::StateInterface> motion_possible_state_;

  rclcpp::Node::SharedPtr client_node_;
  rclcpp::executors::MultiThreadedExecutor executor_;
  std::thread executor_thread_;
  rclcpp::Time time_;
};

TEST_F(ScaledJointTrajectoryControllerTest, ScalingIsRobotScaleTimesTopicValue)
{
  set(robot_scaling_state_, 0.5);
  update();
  EXPECT_DOUBLE_EQ(controller_->get_scaling_factor(), 0.5);

  auto publisher = client_node_->create_publisher<std_msgs::msg::Int32>("speed_scaling_factor", 10);
  std_msgs::msg::Int32 msg;
  msg.data = 50;
  for (int i = 0; i < 20 && publisher->get_subscription_count() == 0; ++i)
  {
    std::this_thread::sleep_for(50ms);
  }
  publisher->publish(msg);
  std::this_thread::sleep_for(200ms);

  // The first order lag filter moves towards the new topic value.
  update();
  EXPECT_LT(controller_->get_scaling_factor(), 0.5);
  EXPECT_GT(controller_->get_scaling_factor(), 0.25);
  update(200);
  EXPECT_NEAR(controller_->get_scaling_factor(), 0.25, 1e-3);

  // Values outside 0 to 100 % are clamped.
  msg.data = 150;
  publisher->publish(msg);
  std::this_thread::sleep_for(200ms);
  update(200);
  EXPECT_NEAR(controller_->get_scaling_factor(), 0.5, 1e-3);
}

TEST_F(ScaledJointTrajectoryControllerTest, AbortGoalAndHoldWhenMotionIsNotPossible)
{
  update();

  auto action_client = rclcpp_action::create_client<FollowJTrajAction>(client_node_, "/test_scaled_jtc/"
                                                                                     "follow_joint_trajectory");
  ASSERT_TRUE(action_client->wait_for_action_server(5s));

  FollowJTrajAction::Goal goal;
  goal.trajectory.joint_names = { "joint1" };
  goal.trajectory.points.resize(1);
  goal.trajectory.points[0].positions = { 1.0 };
  goal.trajectory.points[0].time_from_start = rclcpp::Duration::from_seconds(1.0);

  std::promise<rclcpp_action::ClientGoalHandle<FollowJTrajAction>::WrappedResult> result_promise;
  auto result_future = result_promise.get_future();
  rclcpp_action::Client<FollowJTrajAction>::SendGoalOptions options;
  options.result_callback = [&result_promise](const auto& result) { result_promise.set_value(result); };
  auto goal_handle_future = action_client->async_send_goal(goal, options);
  ASSERT_EQ(goal_handle_future.wait_for(5s), std::future_status::ready);
  ASSERT_NE(goal_handle_future.get(), nullptr) << "Goal was rejected";

  // The robot moves.
  update(20);
  const double position_before_stop = get(position_command_);
  EXPECT_GT(position_before_stop, 0.0);
  EXPECT_LT(position_before_stop, 1.0);

  // Motion is not possible: abort the goal and do not write commands.
  set(motion_possible_state_, 0.0);
  update(10);
  ASSERT_EQ(result_future.wait_for(5s), std::future_status::ready);
  const auto result = result_future.get();
  EXPECT_EQ(result.code, rclcpp_action::ResultCode::ABORTED);
  EXPECT_EQ(result.result->error_code, FollowJTrajAction::Result::INVALID_GOAL);
  EXPECT_EQ(result.result->error_string, "Aborted: STMO is inactive (!motion_possible)");
  EXPECT_DOUBLE_EQ(get(position_command_), position_before_stop);

  // Motion is possible again: hold the current position.
  set(position_state_, 0.3);
  set(motion_possible_state_, 1.0);
  update(50);
  EXPECT_DOUBLE_EQ(get(position_command_), 0.3);
}

}  // namespace

int main(int argc, char** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);
  const int result = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return result;
}
