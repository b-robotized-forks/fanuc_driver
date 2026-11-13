// SPDX-License-Identifier: Apache-2.0

#include <gtest/gtest.h>

#include <chrono>
#include <future>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include "control_msgs/action/follow_joint_trajectory.hpp"
#include "fanuc_controllers/scaled_joint_trajectory_controller.hpp"
#include "hardware_interface/resource_manager.hpp"
#include "lifecycle_msgs/msg/state.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

using namespace std::chrono_literals;
using FollowJTrajAction = control_msgs::action::FollowJointTrajectory;

namespace
{

// FANUC mock hardware. With mock_gpio_commands, every GPIO state has a command interface of the
// same name, which the mock copies to the state in read().
constexpr auto kRobotDescription = R"(<?xml version="1.0"?>
<robot name="mock_fanuc">
  <link name="base_link"/>
  <link name="link1"/>
  <joint name="J1" type="revolute">
    <parent link="base_link"/>
    <child link="link1"/>
    <axis xyz="0 0 1"/>
    <limit lower="-3.0" upper="3.0" effort="10.0" velocity="10.0"/>
  </joint>
  <ros2_control name="fanuc" type="system">
    <hardware>
      <plugin>fanuc_robot_driver/FanucMockHardwareInterface</plugin>
      <param name="mock_gpio_commands">true</param>
    </hardware>
    <joint name="J1">
      <command_interface name="position"/>
      <state_interface name="position">
        <param name="initial_value">0.0</param>
      </state_interface>
    </joint>
    <gpio name="Status">
      <state_interface name="collaborative_speed_scaling">
        <param name="initial_value">1.0</param>
      </state_interface>
      <state_interface name="motion_possible">
        <param name="initial_value">1.0</param>
      </state_interface>
    </gpio>
  </ros2_control>
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

class ScaledJointTrajectoryControllerMockTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    resource_manager_ = std::make_unique<hardware_interface::ResourceManager>(
        kRobotDescription, std::make_shared<rclcpp::Clock>(), rclcpp::get_logger("mock_hardware"), true);

    controller_ = std::make_shared<TestableScaledJointTrajectoryController>();
    controller_interface::ControllerInterfaceParams params;
    params.controller_name = "scaled_jtc";
    params.robot_description = kRobotDescription;
    params.update_rate = 100;
    params.controller_manager_update_rate = 100;
    params.node_options.parameter_overrides({ { "joints", std::vector<std::string>{ "J1" } },
                                              { "command_interfaces", std::vector<std::string>{ "position" } },
                                              { "state_interfaces", std::vector<std::string>{ "position" } } });
    ASSERT_EQ(controller_->init(params), controller_interface::return_type::OK);
    ASSERT_EQ(controller_->configure().id(), lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE);

    std::vector<hardware_interface::LoanedCommandInterface> command_interfaces;
    for (const auto& name : controller_->command_interface_configuration().names)
    {
      command_interfaces.emplace_back(resource_manager_->claim_command_interface(name));
    }
    std::vector<hardware_interface::LoanedStateInterface> state_interfaces;
    for (const auto& name : controller_->state_interface_configuration().names)
    {
      state_interfaces.emplace_back(resource_manager_->claim_state_interface(name));
    }
    controller_->assign_interfaces(std::move(command_interfaces), std::move(state_interfaces));
    ASSERT_EQ(controller_->get_node()->activate().id(), lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE);

    client_node_ = std::make_shared<rclcpp::Node>("test_client");
    executor_.add_node(controller_->get_node()->get_node_base_interface());
    executor_.add_node(client_node_);
    executor_thread_ = std::thread([this]() { executor_.spin(); });
    // cancel() in TearDown() has no effect before spin() starts.
    while (!executor_.is_spinning())
    {
      std::this_thread::sleep_for(1ms);
    }

    time_ = controller_->get_node()->now();
  }

  void TearDown() override
  {
    executor_.cancel();
    executor_thread_.join();
    controller_->get_node()->deactivate();
  }

  /// Set a FANUC status value on the mock robot.
  void set_robot_status(const std::string& name, double value)
  {
    auto command = resource_manager_->claim_command_interface("Status/" + name);
    ASSERT_TRUE(command.set_value(value));
  }

  double joint_command()
  {
    return resource_manager_->claim_state_interface("J1/position").get_optional().value();
  }

  /// One control cycle as in the controller manager: read, update, write.
  void cycle(size_t count = 1)
  {
    const rclcpp::Duration period = rclcpp::Duration::from_seconds(0.01);
    for (size_t i = 0; i < count; ++i)
    {
      time_ += period;
      ASSERT_EQ(resource_manager_->read(time_, period).result, hardware_interface::return_type::OK);
      ASSERT_EQ(controller_->update(time_, period), controller_interface::return_type::OK);
      ASSERT_EQ(resource_manager_->write(time_, period).result, hardware_interface::return_type::OK);
    }
  }

  std::unique_ptr<hardware_interface::ResourceManager> resource_manager_;
  std::shared_ptr<TestableScaledJointTrajectoryController> controller_;
  rclcpp::Node::SharedPtr client_node_;
  rclcpp::executors::MultiThreadedExecutor executor_;
  std::thread executor_thread_;
  rclcpp::Time time_;
};

TEST_F(ScaledJointTrajectoryControllerMockTest, UsesTheRobotSpeedScaling)
{
  cycle();
  EXPECT_DOUBLE_EQ(controller_->get_scaling_factor(), 1.0);

  set_robot_status("collaborative_speed_scaling", 0.4);
  cycle();
  EXPECT_DOUBLE_EQ(controller_->get_scaling_factor(), 0.4);
}

TEST_F(ScaledJointTrajectoryControllerMockTest, AbortsTheGoalWhileMotionIsNotPossible)
{
  cycle();

  auto action_client = rclcpp_action::create_client<FollowJTrajAction>(client_node_, "/scaled_jtc/"
                                                                                     "follow_joint_trajectory");
  ASSERT_TRUE(action_client->wait_for_action_server(5s));

  FollowJTrajAction::Goal goal;
  goal.trajectory.joint_names = { "J1" };
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
  cycle(20);

  // The robot reports that motion is not possible: the goal is aborted, the robot stays.
  set_robot_status("motion_possible", 0.0);
  cycle(10);
  ASSERT_EQ(result_future.wait_for(5s), std::future_status::ready);
  const auto result = result_future.get();
  EXPECT_EQ(result.code, rclcpp_action::ResultCode::ABORTED);
  EXPECT_EQ(result.result->error_string, "Aborted: STMO is inactive (!motion_possible)");
  const double stop_position = joint_command();
  EXPECT_GT(stop_position, 0.0);
  EXPECT_LT(stop_position, 1.0);
  cycle(10);
  EXPECT_DOUBLE_EQ(joint_command(), stop_position);

  // Motion is possible again: the robot holds its position.
  set_robot_status("motion_possible", 1.0);
  cycle(50);
  EXPECT_NEAR(joint_command(), stop_position, 1e-9);
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
