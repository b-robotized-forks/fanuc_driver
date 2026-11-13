// SPDX-License-Identifier: Apache-2.0

#pragma once

#include <cstdint>
#include <memory>
#include <mutex>
#include <optional>
#include <string>

#include "fanuc_robot_driver/inert_rmi_connection.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "mock_components/generic_system.hpp"
#include "rclcpp/duration.hpp"
#include "rclcpp/time.hpp"
#include "rclcpp_lifecycle/state.hpp"

namespace fanuc_robot_driver
{

/// GPIO and command interface that inject an alarm into the mock controller.
constexpr auto kMockErrorInjectionName = "MockErrorInjection";
constexpr auto kMockErrorInjectionAlarmType = "alarm";

/**
 * Turn an injected alarm value into the controller's alarm code, e.g. 11050 into "SRVO-050".
 *
 * The value is facility * 1000 + number, with FANUC's facility codes. Returns nothing for
 * zero, a non-integer value, or an unknown facility.
 */
std::optional<std::string> AlarmCodeFromValue(double value);

/// The alarm the mock controller holds, shared by the hardware and its RMI connection.
class MockControllerAlarm
{
public:
  void raise(const std::string& code);
  void clear();
  std::optional<std::string> code() const;

private:
  mutable std::mutex mutex_;
  std::optional<std::string> code_;
};

/// RMI connection of the mock controller: FRC_Reset clears the alarm, the status reports it.
class MockRMIConnection : public InertRMIConnection
{
public:
  explicit MockRMIConnection(std::shared_ptr<MockControllerAlarm> alarm);

  rmi::ResetRobotPacket::Response reset(std::optional<double> timeout) override;
  rmi::ReadErrorPacket::Response readError(std::optional<double> timeout) override;
  rmi::ReadErrorPacket::Response readError(std::optional<double> timeout, std::optional<uint8_t> count) override;
  rmi::GetExtendedStatusPacket::Response getExtendedStatus(std::optional<double> timeout) override;

private:
  std::shared_ptr<MockControllerAlarm> alarm_;
};

/**
 * Mock FANUC robot that behaves like the physical one towards the controllers.
 *
 * It installs MockRMIConnection as the RMI singleton, so FanucGPIOController's services
 * answer as on the robot. A new non-zero value on MockErrorInjection/alarm raises an alarm
 * once; Status/in_error follows the alarm until FRC_Reset clears it.
 */
class FanucMockHardwareInterface : public mock_components::GenericSystem
{
public:
  hardware_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State& previous_state) override;

  hardware_interface::return_type read(const rclcpp::Time& time, const rclcpp::Duration& period) override;

  hardware_interface::return_type write(const rclcpp::Time& time, const rclcpp::Duration& period) override;

private:
  std::shared_ptr<MockControllerAlarm> alarm_ = std::make_shared<MockControllerAlarm>();
  double last_injected_value_ = 0.0;
};

}  // namespace fanuc_robot_driver
