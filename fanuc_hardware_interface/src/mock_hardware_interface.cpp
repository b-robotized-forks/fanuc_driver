// SPDX-FileCopyrightText: 2026, b»robotized group
//
// SPDX-License-Identifier: Apache-2.0

#include "fanuc_robot_driver/mock_hardware_interface.hpp"

#include <cmath>
#include <cstdio>
#include <map>
#include <utility>

#include "fanuc_client/fanuc_client.hpp"
#include "fanuc_robot_driver/constants.hpp"
#include "rclcpp/logging.hpp"

namespace fanuc_robot_driver
{

namespace
{

// FANUC facility codes of the alarm families the mock can raise.
const std::map<int, std::string> kAlarmFacilities = {
  { 3, "PROG" }, { 11, "SRVO" }, { 12, "INTP" }, { 15, "MOTN" }, { 24, "SYST" }, { 67, "HOST" },
};

std::string StatusInterface(const char* type)
{
  return std::string(kRobotStatusInterfaceName) + "/" + type;
}

}  // namespace

std::optional<std::string> AlarmCodeFromValue(double value)
{
  if (!std::isfinite(value) || value <= 0.0 || value != std::floor(value))
  {
    return std::nullopt;
  }
  const auto encoded = static_cast<long>(value);
  const auto facility = kAlarmFacilities.find(static_cast<int>(encoded / 1000));
  if (facility == kAlarmFacilities.end())
  {
    return std::nullopt;
  }
  char code[16];
  std::snprintf(code, sizeof(code), "%s-%03ld", facility->second.c_str(), encoded % 1000);
  return std::string(code);
}

void MockControllerAlarm::raise(const std::string& code)
{
  std::scoped_lock lock(mutex_);
  code_ = code;
}

void MockControllerAlarm::clear()
{
  std::scoped_lock lock(mutex_);
  code_.reset();
}

std::optional<std::string> MockControllerAlarm::code() const
{
  std::scoped_lock lock(mutex_);
  return code_;
}

MockRMIConnection::MockRMIConnection(std::shared_ptr<MockControllerAlarm> alarm) : alarm_(std::move(alarm))
{
}

rmi::ResetRobotPacket::Response MockRMIConnection::reset(std::optional<double> /*timeout*/)
{
  alarm_->clear();
  return {};
}

rmi::ReadErrorPacket::Response MockRMIConnection::readError(std::optional<double> timeout)
{
  return readError(timeout, 1);
}

rmi::ReadErrorPacket::Response MockRMIConnection::readError(std::optional<double> /*timeout*/,
                                                            std::optional<uint8_t> /*count*/)
{
  rmi::ReadErrorPacket::Response response{};
  const auto code = alarm_->code();
  if (code.has_value())
  {
    response.Count = 1;
    response.ErrorData = code.value() + " Mock alarm";
  }
  return response;
}

rmi::GetExtendedStatusPacket::Response MockRMIConnection::getExtendedStatus(std::optional<double> /*timeout*/)
{
  rmi::GetExtendedStatusPacket::Response response{};
  response.ErrorCode = alarm_->code();
  return response;
}

hardware_interface::CallbackReturn
FanucMockHardwareInterface::on_configure(const rclcpp_lifecycle::State& previous_state)
{
  const auto result = GenericSystem::on_configure(previous_state);
  if (result != hardware_interface::CallbackReturn::SUCCESS)
  {
    return result;
  }
  // The physical driver opens its RMI session here too; the controllers reach it through the singleton.
  fanuc_client::RMISingleton::setRMIInstance(std::make_unique<MockRMIConnection>(alarm_));
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type FanucMockHardwareInterface::read(const rclcpp::Time& time,
                                                                 const rclcpp::Duration& period)
{
  const auto result = GenericSystem::read(time, period);
  const auto in_error = StatusInterface(kStatusInErrorType);
  if (has_state(in_error))
  {
    set_state(in_error, alarm_->code().has_value() ? 1.0 : 0.0);
  }
  return result;
}

hardware_interface::return_type FanucMockHardwareInterface::write(const rclcpp::Time& /*time*/,
                                                                  const rclcpp::Duration& /*period*/)
{
  const auto injection = std::string(kMockErrorInjectionName) + "/" + kMockErrorInjectionAlarmType;
  if (!has_command(injection))
  {
    return hardware_interface::return_type::OK;
  }
  // A forwarding controller repeats its last command every cycle; only a new value raises an alarm.
  const double value = get_command(injection);
  if (!std::isfinite(value) || value == last_injected_value_)
  {
    return hardware_interface::return_type::OK;
  }
  last_injected_value_ = value;
  const auto code = AlarmCodeFromValue(value);
  if (code.has_value())
  {
    alarm_->raise(code.value());
    RCLCPP_WARN(get_logger(), "Mock alarm raised: %s", code->c_str());
  }
  else if (value != 0.0)
  {
    RCLCPP_ERROR(get_logger(), "Mock alarm value %.0f is not facility * 1000 + number; ignored.", value);
  }
  return hardware_interface::return_type::OK;
}

}  // namespace fanuc_robot_driver

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(fanuc_robot_driver::FanucMockHardwareInterface, hardware_interface::SystemInterface)
