// SPDX-FileCopyrightText: 2026, b»robotized group
//
// SPDX-License-Identifier: Apache-2.0

#pragma once

#include <cstdint>
#include <optional>
#include <string>
#include <variant>
#include <vector>

#include "rmi/rmi.hpp"

namespace fanuc_robot_driver
{

// The RMI interface is too wide to stub by hand; every method answers with a
// default-constructed value.
#define INERT_RESPONSE(ret, ...)                                                                                       \
  ret __VA_ARGS__ override                                                                                             \
  {                                                                                                                    \
    return {};                                                                                                         \
  }
#define INERT_VOID(...)                                                                                                \
  void __VA_ARGS__ override                                                                                            \
  {                                                                                                                    \
  }

/// RMI fake answering every request with a default-constructed response.
class InertRMIConnection : public rmi::RMIConnectionInterface
{
public:
  ~InertRMIConnection() override = default;

  INERT_RESPONSE(rmi::ConnectROS2Packet::Response, connect(std::optional<double>))
  INERT_RESPONSE(rmi::DisconnectPacket::Response, disconnect(std::optional<double>))
  INERT_RESPONSE(rmi::InitializePacket::Response, initializeRemoteMotion(std::optional<double>))
  INERT_RESPONSE(rmi::InitializePacket::Response,
                 initializeRemoteMotion(std::optional<double>, std::optional<uint8_t>,
                                        const std::optional<std::string>&, const std::optional<std::string>&))
  INERT_RESPONSE(rmi::ProgramCallPacket::Response, programCall(const std::string&, std::optional<double>))
  INERT_RESPONSE(rmi::ProgramCallPacket::Response,
                 programCall(const std::string&, std::optional<double>, const std::vector<rmi::RMICallParam>&))
  INERT_RESPONSE(rmi::ProgramCallPacket::Request, programCallNonBlocking(const std::string&))
  INERT_RESPONSE(rmi::ProgramCallPacket::Request,
                 programCallNonBlocking(const std::string&, const std::vector<rmi::RMICallParam>&))
  INERT_RESPONSE(rmi::StatusRequestPacket::Response, getStatus(std::optional<double>))
  INERT_RESPONSE(rmi::SetSpeedOverridePacket::Response, setSpeedOverride(int, std::optional<double>))
  INERT_RESPONSE(rmi::AbortPacket::Response, abort(std::optional<double>))
  INERT_RESPONSE(rmi::PausePacket::Response, pause(std::optional<double>))
  INERT_RESPONSE(rmi::ContinuePacket::Response, resume(std::optional<double>))
  INERT_RESPONSE(rmi::ResetRobotPacket::Response, reset(std::optional<double>))
  INERT_RESPONSE(rmi::ReadErrorPacket::Response, readError(std::optional<double>))
  INERT_RESPONSE(rmi::ReadErrorPacket::Response, readError(std::optional<double>, std::optional<uint8_t>))
  INERT_RESPONSE(rmi::GetUFrameToolFramePacket::Response, getUFrameUTool(std::optional<double>, std::optional<uint8_t>))
  INERT_RESPONSE(rmi::SetUFrameToolFramePacket::Response,
                 setUFrameUTool(int, int, std::optional<double>, std::optional<uint8_t>))
  INERT_RESPONSE(rmi::ReadUFrameDataPacket::Response, readUFrameData(int, std::optional<double>, std::optional<uint8_t>))
  INERT_RESPONSE(rmi::WriteUFrameDataPacket::Response,
                 writeUFrameData(int, rmi::FrameData, std::optional<double>, std::optional<uint8_t>))
  INERT_RESPONSE(rmi::ReadUToolDataPacket::Response, readUToolData(int, std::optional<double>, std::optional<uint8_t>))
  INERT_RESPONSE(rmi::WriteUToolDataPacket::Response,
                 writeUToolData(int, rmi::FrameData, std::optional<double>, std::optional<uint8_t>))
  INERT_RESPONSE(rmi::WritePositionRegisterPacket::Response,
                 writePositionRegister(int, const std::string&, const rmi::ConfigurationData&, const rmi::PositionData&,
                                       const rmi::JointAngleData&, std::optional<double>))
  INERT_RESPONSE(rmi::ReadPositionRegisterPacket::Response, readPositionRegister(int, std::optional<double>))
  INERT_RESPONSE(rmi::ReadNumericRegisterPacket::Response, readNumericRegister(int, std::optional<double>))
  INERT_RESPONSE(rmi::WriteNumericRegisterPacket::Response,
                 writeNumericRegister(int, std::variant<int, float>, std::optional<double>))
  INERT_RESPONSE(rmi::ReadDigitalInputPortPacket::Response, readDigitalInputPort(uint16_t, std::optional<double>))
  INERT_RESPONSE(rmi::WriteDigitalOutputPacket::Response, writeDigitalOutputPort(uint16_t, bool, std::optional<double>))
  INERT_RESPONSE(rmi::ReadIOPortPacket::Response, readIOPort(const std::string&, int, std::optional<double>))
  INERT_RESPONSE(rmi::WriteIOPortPacket::Response,
                 writeIOPort(int, const std::string&, std::variant<int, float>, std::optional<double>))
  INERT_RESPONSE(rmi::ReadVariablePacket::Response, readVariablePacket(const std::string&, std::optional<double>))
  INERT_RESPONSE(rmi::WriteVariablePacket::Response,
                 writeVariablePacket(const std::string&, std::variant<int, float>, std::optional<double>))
  INERT_RESPONSE(rmi::GetExtendedStatusPacket::Response, getExtendedStatus(std::optional<double>))
  INERT_RESPONSE(rmi::SetPayloadPacket::Response, setPayloadSchedule(uint8_t, std::optional<double>))
  INERT_RESPONSE(rmi::SetPayloadValuePacket::Response,
                 setPayloadValue(uint8_t, float, float, float, float, bool, float, float, float, std::optional<double>))
  INERT_RESPONSE(rmi::SetPayloadCompPacket::Response,
                 setPayloadComp(uint8_t, float, float, float, float, float, float, float, std::optional<double>))
  INERT_RESPONSE(rmi::GetPayloadPacket::Response, getPayloadSchedule(std::optional<double>))
  INERT_RESPONSE(rmi::GetPayloadValuePacket::Response, getPayloadValue(uint16_t, std::optional<double>))
  INERT_RESPONSE(rmi::GetPayloadCompPacket::Response, getPayloadComp(uint16_t, std::optional<double>))
  INERT_RESPONSE(rmi::JointMotionJRepPacket::Response,
                 sendJointMotion(rmi::JointMotionJRepPacket::Request, std::optional<double>))
  INERT_RESPONSE(rmi::ReadJointAnglesPacket::Response,
                 readJointAngles(const std::optional<uint8_t>&, std::optional<double>))
  INERT_RESPONSE(std::optional<rmi::SystemFaultPacket>, checkSystemFault())
  INERT_RESPONSE(std::optional<rmi::TimeoutTerminatePacket>, checkTimeoutTerminate())
  INERT_RESPONSE(std::optional<rmi::CommunicationPacket>, checkCommunicationPacket())
  INERT_RESPONSE(std::optional<rmi::UnknownPacket>, checkUnknownPacket())
  INERT_RESPONSE(rmi::GetCartesianPositionPacket::Response,
                 getCartesianPosition(std::optional<double>, std::optional<uint8_t>))
  INERT_RESPONSE(rmi::GetTCPSpeedPacket::Response, getTCPSpeed(std::optional<double>, std::optional<uint8_t>))
  INERT_RESPONSE(std::optional<rmi::InstructionResponse>, getLastInstructionResponse())
  INERT_VOID(sendRMIPacketNonBlocking(rmi::JointMotionPacket::Request&))
  INERT_VOID(sendRMIPacketNonBlocking(rmi::JointMotionJRepPacket::Request&))
  INERT_VOID(sendRMIPacketNonBlocking(rmi::JointRelativePacket::Request&))
  INERT_VOID(sendRMIPacketNonBlocking(rmi::JointRelativeJRepPacket::Request&))
  INERT_VOID(sendRMIPacketNonBlocking(rmi::LinearMotionPacket::Request&))
  INERT_VOID(sendRMIPacketNonBlocking(rmi::LinearMotionJRepPacket::Request&))
  INERT_VOID(sendRMIPacketNonBlocking(rmi::LinearRelativePacket::Request&))
  INERT_VOID(sendRMIPacketNonBlocking(rmi::LinearRelativeJRepPacket::Request&))
  INERT_VOID(sendRMIPacketNonBlocking(rmi::CircularMotionPacket::Request&))
  INERT_VOID(sendRMIPacketNonBlocking(rmi::CircularRelativePacket::Request&))
  INERT_VOID(sendRMIPacketNonBlocking(rmi::SplineMotionPacket::Request&))
  INERT_VOID(sendRMIPacketNonBlocking(rmi::SplineMotionJRepPacket::Request&))
  INERT_VOID(sendRMIPacketNonBlocking(rmi::WaitForDINPacket::Request&))
  INERT_VOID(sendRMIPacketNonBlocking(rmi::SetUFramePacket::Request&))
  INERT_VOID(sendRMIPacketNonBlocking(rmi::SetToolFramePacket::Request&))
  INERT_VOID(sendRMIPacketNonBlocking(rmi::WaitForTimePacket::Request&))
  INERT_VOID(sendRMIPacketNonBlocking(rmi::SetPayloadInstructionPacket::Request&))
  INERT_RESPONSE(std::string, getErrorMessageString(uint32_t))
  INERT_RESPONSE(int32_t, getRemainingBuffuerSize())
  INERT_VOID(setEncoding(const std::string&))
};

#undef INERT_RESPONSE
#undef INERT_VOID

}  // namespace fanuc_robot_driver
