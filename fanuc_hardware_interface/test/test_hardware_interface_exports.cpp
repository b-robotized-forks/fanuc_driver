// SPDX-FileCopyrightText: 2026, FANUC America Corporation
// SPDX-FileCopyrightText: 2026, FANUC CORPORATION
//
// SPDX-License-Identifier: Apache-2.0

// Regression test for the deprecated-Handle-API migration: the exported state/command
// interface *names* must exactly match what the old raw-pointer export_state_interfaces()/
// export_command_interfaces() used to export, one-for-one. Nothing here exercises hardware -
// only on_init() + export via the hardware_interface::System wrapper (which runs the same
// old-export-empty -> on_export_state_interfaces() fallback the real resource_manager uses).
//
// No gpio_configuration parameter is set, so this only covers the joint and fixed
// (status/connection/force) interfaces - not the dynamically-named GPIO ones, whose names
// depend on a runtime-parsed config file rather than anything a unit test can enumerate ahead
// of time.

#include <gmock/gmock.h>

#include <memory>
#include <string>
#include <vector>

#include "fanuc_robot_driver/hardware_interface.hpp"
#include "hardware_interface/system.hpp"
#include "hardware_interface/types/hardware_component_params.hpp"

namespace
{
constexpr size_t kJointCount = 6;

hardware_interface::HardwareInfo BuildTestHardwareInfo()
{
  hardware_interface::HardwareInfo info;
  // Required for HardwareComponentInterface::on_init() to parse info.joints into its internal
  // joint_state_interfaces_/joint_command_interfaces_ maps at all.
  info.type = "system";

  auto make_interface = [](const std::string& name)
  {
    hardware_interface::InterfaceInfo iface{};
    iface.name = name;
    return iface;
  };

  for (size_t i = 1; i <= kJointCount; ++i)
  {
    hardware_interface::ComponentInfo joint;
    joint.name = "J" + std::to_string(i);
    joint.command_interfaces = { make_interface(hardware_interface::HW_IF_POSITION) };
    joint.state_interfaces = { make_interface(hardware_interface::HW_IF_POSITION),
                                make_interface(hardware_interface::HW_IF_VELOCITY) };
    info.joints.push_back(joint);
  }
  return info;
}

std::vector<std::string> GetNames(const std::vector<hardware_interface::StateInterface::ConstSharedPtr>& interfaces)
{
  std::vector<std::string> names;
  names.reserve(interfaces.size());
  for (const auto& interface : interfaces)
  {
    names.push_back(interface->get_name());
  }
  return names;
}

std::vector<std::string> GetNames(const std::vector<hardware_interface::CommandInterface::SharedPtr>& interfaces)
{
  std::vector<std::string> names;
  names.reserve(interfaces.size());
  for (const auto& interface : interfaces)
  {
    names.push_back(interface->get_name());
  }
  return names;
}

class FanucHardwareInterfaceExportTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    hardware_interface::HardwareComponentParams params;
    params.hardware_info = BuildTestHardwareInfo();
    params.clock = std::make_shared<rclcpp::Clock>();
    params.logger = rclcpp::get_logger("test_fanuc_hardware_interface_exports");

    hw_ =
        std::make_unique<hardware_interface::System>(std::make_unique<fanuc_robot_driver::FanucHardwareInterface>());
    const auto state = hw_->initialize(params);
    ASSERT_EQ(state.id(), lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED)
        << "on_init() did not succeed - check the test HardwareInfo matches what "
           "FanucHardwareInterface::on_init() validates.";
  }

  std::unique_ptr<hardware_interface::System> hw_;
};

// Interfaces exported by the removed raw-pointer export_state_interfaces(), for comparison:
//   per joint: <joint>/position, <joint>/velocity
//   plus:      Status/*, ConnectionStatus/is_connected, Force/*, ft_sensor/*
TEST_F(FanucHardwareInterfaceExportTest, ExportsExpectedStateInterfaces)
{
  const auto exported_names = GetNames(hw_->export_state_interfaces());

  std::vector<std::string> expected_names;
  for (size_t i = 1; i <= kJointCount; ++i)
  {
    const std::string joint = "J" + std::to_string(i);
    expected_names.push_back(joint + "/" + hardware_interface::HW_IF_POSITION);
    expected_names.push_back(joint + "/" + hardware_interface::HW_IF_VELOCITY);
  }
  for (const char* name : { "in_error", "tp_enabled", "e_stopped", "motion_possible", "contact_stop_mode",
                             "collaborative_speed_scaling" })
  {
    expected_names.push_back(std::string("Status/") + name);
  }
  expected_names.push_back("ConnectionStatus/is_connected");
  for (const char* name : { "force_x", "force_y", "force_z", "moment_x", "moment_y", "moment_z", "fs_type" })
  {
    expected_names.push_back(std::string("Force/") + name);
  }
  for (const char* name : { "force.x", "force.y", "force.z", "torque.x", "torque.y", "torque.z" })
  {
    expected_names.push_back(std::string("ft_sensor/") + name);
  }

  EXPECT_THAT(exported_names, ::testing::UnorderedElementsAreArray(expected_names));
}

// Interfaces exported by the removed raw-pointer export_command_interfaces(), for comparison:
//   per joint: <joint>/position
TEST_F(FanucHardwareInterfaceExportTest, ExportsExpectedCommandInterfaces)
{
  const auto exported_names = GetNames(hw_->export_command_interfaces());

  std::vector<std::string> expected_names;
  for (size_t i = 1; i <= kJointCount; ++i)
  {
    expected_names.push_back("J" + std::to_string(i) + "/" + hardware_interface::HW_IF_POSITION);
  }

  EXPECT_THAT(exported_names, ::testing::UnorderedElementsAreArray(expected_names));
}
}  // namespace

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  testing::InitGoogleMock(&argc, argv);
  int result = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return result;
}
