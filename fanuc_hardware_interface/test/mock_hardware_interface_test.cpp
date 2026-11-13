// SPDX-FileCopyrightText: 2026, b»robotized group
//
// SPDX-License-Identifier: Apache-2.0

#include <gtest/gtest.h>

#include <memory>
#include <string>

#include "fanuc_client/fanuc_client.hpp"
#include "fanuc_robot_driver/mock_hardware_interface.hpp"
#include "hardware_interface/resource_manager.hpp"
#include "rclcpp/clock.hpp"
#include "rclcpp/duration.hpp"
#include "rclcpp/logger.hpp"
#include "rclcpp/time.hpp"

namespace fanuc_robot_driver
{
namespace
{

TEST(AlarmCodeFromValue, DecodesFacilityAndNumber)
{
  EXPECT_EQ(AlarmCodeFromValue(11050.0), "SRVO-050");
  EXPECT_EQ(AlarmCodeFromValue(15018.0), "MOTN-018");
  EXPECT_EQ(AlarmCodeFromValue(3001.0), "PROG-001");
}

TEST(AlarmCodeFromValue, RejectsValuesThatAreNoAlarm)
{
  EXPECT_FALSE(AlarmCodeFromValue(0.0).has_value());
  EXPECT_FALSE(AlarmCodeFromValue(-11050.0).has_value());
  EXPECT_FALSE(AlarmCodeFromValue(11050.5).has_value());
  EXPECT_FALSE(AlarmCodeFromValue(99050.0).has_value());
  EXPECT_FALSE(AlarmCodeFromValue(std::nan("")).has_value());
}

TEST(MockRMIConnection, ReportsTheAlarmUntilReset)
{
  auto alarm = std::make_shared<MockControllerAlarm>();
  MockRMIConnection rmi(alarm);
  alarm->raise("SRVO-050");

  EXPECT_EQ(rmi.getExtendedStatus(1.0).ErrorCode, "SRVO-050");
  const auto error = rmi.readError(1.0, 1);
  EXPECT_EQ(error.ErrorID, 0);
  EXPECT_EQ(error.Count, 1);
  EXPECT_EQ(error.ErrorData.rfind("SRVO-050", 0), 0u);

  EXPECT_EQ(rmi.reset(1.0).ErrorID, 0);
  EXPECT_FALSE(alarm->code().has_value());
  EXPECT_FALSE(rmi.getExtendedStatus(1.0).ErrorCode.has_value());
  EXPECT_FALSE(rmi.readError(1.0, 1).Count.has_value());
}

constexpr auto kUrdf = R"(
<robot name="mock_fanuc">
  <link name="base"/>
  <ros2_control name="fanuc" type="system">
    <hardware>
      <plugin>fanuc_robot_driver/FanucMockHardwareInterface</plugin>
    </hardware>
    <gpio name="Status">
      <state_interface name="in_error">
        <param name="initial_value">0</param>
      </state_interface>
    </gpio>
    <gpio name="MockErrorInjection">
      <command_interface name="alarm"/>
    </gpio>
  </ros2_control>
</robot>
)";

/// The plugin loaded by the resource manager, as ros2_control_node does it.
class FanucMockHardwareInterfaceTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    resource_manager_ = std::make_unique<hardware_interface::ResourceManager>(
        kUrdf, std::make_shared<rclcpp::Clock>(), rclcpp::get_logger("mock_hardware_test"), true);
  }

  void Cycle()
  {
    const rclcpp::Time time(0);
    const rclcpp::Duration period(0, 10'000'000);
    ASSERT_EQ(resource_manager_->write(time, period).result, hardware_interface::return_type::OK);
    ASSERT_EQ(resource_manager_->read(time, period).result, hardware_interface::return_type::OK);
  }

  double InError()
  {
    auto in_error = resource_manager_->claim_state_interface("Status/in_error");
    return in_error.get_optional().value();
  }

  std::unique_ptr<hardware_interface::ResourceManager> resource_manager_;
};

TEST_F(FanucMockHardwareInterfaceTest, StartsWithoutAnAlarm)
{
  Cycle();
  EXPECT_EQ(InError(), 0.0);
}

TEST_F(FanucMockHardwareInterfaceTest, AnInjectedAlarmStaysUntilTheControllerResets)
{
  auto injection = resource_manager_->claim_command_interface("MockErrorInjection/alarm");
  ASSERT_TRUE(injection.set_value(11050.0));
  Cycle();
  EXPECT_EQ(InError(), 1.0);
  EXPECT_EQ(fanuc_client::RMISingleton::getRMIInstance()->getExtendedStatus(1.0).ErrorCode, "SRVO-050");

  fanuc_client::RMISingleton::getRMIInstance()->reset(1.0);
  // The command still holds 11050, as a forwarding controller would repeat it.
  Cycle();
  Cycle();
  EXPECT_EQ(InError(), 0.0);
}

TEST_F(FanucMockHardwareInterfaceTest, TheSameAlarmCanBeRaisedAgainAfterAZero)
{
  auto injection = resource_manager_->claim_command_interface("MockErrorInjection/alarm");
  ASSERT_TRUE(injection.set_value(11050.0));
  Cycle();
  fanuc_client::RMISingleton::getRMIInstance()->reset(1.0);
  ASSERT_TRUE(injection.set_value(0.0));
  Cycle();
  EXPECT_EQ(InError(), 0.0);

  ASSERT_TRUE(injection.set_value(11050.0));
  Cycle();
  EXPECT_EQ(InError(), 1.0);
}

}  // namespace
}  // namespace fanuc_robot_driver
