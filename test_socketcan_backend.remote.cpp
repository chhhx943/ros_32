#include "car_hardware_adapter/socketcan_backend.hpp"

#include <linux/can.h>
#include <linux/can/error.h>

#include <array>
#include <cstdint>
#include <string>
#include <vector>

#include <gtest/gtest.h>

using namespace car_hardware_adapter;

namespace
{

CanFrame valid_command_frame(std::uint32_t id = 0x120U)
{
  return CanFrame{id, false, false, std::vector<std::uint8_t>{1U, 2U, 3U, 4U, 5U, 6U, 7U, 8U}};
}

TEST(SocketCanBackendTest, ConvertsAndValidatesStandardEightByteFrame)
{
  const auto source = valid_command_frame(0x123U);
  can_frame native{};
  std::string reason;

  ASSERT_TRUE(SocketCanBackend::to_native_frame(source, native, reason));
  EXPECT_EQ(native.can_id, 0x123U);
  EXPECT_EQ(native.can_dlc, 8U);
  EXPECT_EQ(native.data[0], 1U);
  EXPECT_EQ(native.data[7], 8U);

  const auto round_trip = SocketCanBackend::from_native_frame(native);
  EXPECT_EQ(round_trip, source);
}

TEST(SocketCanBackendTest, RejectsNonStandardOrNonEightByteFrame)
{
  can_frame native{};
  std::string reason;

  auto extended = valid_command_frame();
  extended.extended = true;
  EXPECT_FALSE(SocketCanBackend::to_native_frame(extended, native, reason));
  EXPECT_EQ(reason, "extended_id");

  auto remote = valid_command_frame();
  remote.rtr = true;
  EXPECT_FALSE(SocketCanBackend::to_native_frame(remote, native, reason));
  EXPECT_EQ(reason, "remote_frame");

  auto invalid_id = valid_command_frame(0x800U);
  EXPECT_FALSE(SocketCanBackend::to_native_frame(invalid_id, native, reason));
  EXPECT_EQ(reason, "invalid_id");

  auto invalid_dlc = valid_command_frame();
  invalid_dlc.data.pop_back();
  EXPECT_FALSE(SocketCanBackend::to_native_frame(invalid_dlc, native, reason));
  EXPECT_EQ(reason, "invalid_dlc");
}

TEST(SocketCanBackendTest, ClassifiesBusOffErrorFrame)
{
  can_frame frame{};
  frame.can_id = CAN_ERR_FLAG | CAN_ERR_BUSOFF;

  const auto result = SocketCanBackend::classify_error_frame(frame);
  EXPECT_EQ(result.error_class, TransportErrorClass::BusOff);
  EXPECT_TRUE(result.disconnect);
  EXPECT_EQ(result.reason, "bus_off");
}

TEST(SocketCanBackendTest, ClassifiesErrorPassiveAndWarnings)
{
  can_frame passive{};
  passive.can_id = CAN_ERR_FLAG | CAN_ERR_CRTL;
  passive.data[1] = CAN_ERR_CRTL_TX_PASSIVE;
  const auto passive_result = SocketCanBackend::classify_error_frame(passive);
  EXPECT_EQ(passive_result.error_class, TransportErrorClass::ErrorPassive);
  EXPECT_FALSE(passive_result.disconnect);

  can_frame warning{};
  warning.can_id = CAN_ERR_FLAG | CAN_ERR_CRTL;
  warning.data[1] = CAN_ERR_CRTL_TX_WARNING;
  const auto warning_result = SocketCanBackend::classify_error_frame(warning);
  EXPECT_EQ(warning_result.error_class, TransportErrorClass::Warning);
  EXPECT_FALSE(warning_result.disconnect);
}

TEST(SocketCanBackendTest, ClassifiesAckAndTimeoutAsTxFailure)
{
  can_frame ack{};
  ack.can_id = CAN_ERR_FLAG | CAN_ERR_ACK;
  EXPECT_EQ(
    SocketCanBackend::classify_error_frame(ack).error_class,
    TransportErrorClass::TxFailure);

  can_frame timeout{};
  timeout.can_id = CAN_ERR_FLAG | CAN_ERR_TX_TIMEOUT;
  EXPECT_EQ(
    SocketCanBackend::classify_error_frame(timeout).error_class,
    TransportErrorClass::TxFailure);
}

TEST(SocketCanBackendTest, ErrorFrameActivatesSafeStopAndConsumesIt)
{
  SocketCanBackend backend;
  can_frame frame{};
  frame.can_id = CAN_ERR_FLAG | CAN_ERR_BUSOFF;

  const auto before = backend.health();
  EXPECT_TRUE(before.safe_stop_active);
  backend.process_error_frame(frame);
  const auto after = backend.health();
  EXPECT_TRUE(after.safe_stop_active);
  EXPECT_FALSE(after.connected);
  EXPECT_FALSE(after.writable);
  EXPECT_EQ(after.error_class, TransportErrorClass::BusOff);
  EXPECT_EQ(after.error_count, 1U);
  EXPECT_EQ(after.last_error, "bus_off");
}

TEST(SocketCanBackendTest, AnyCanErrorDisablesWritesUntilReconnect)
{
  SocketCanBackend backend;
  can_frame frame{};
  frame.can_id = CAN_ERR_FLAG | CAN_ERR_CRTL;
  frame.data[1] = CAN_ERR_CRTL_TX_WARNING;

  backend.process_error_frame(frame);
  const auto health = backend.health();
  EXPECT_FALSE(health.writable);
  EXPECT_TRUE(health.safe_stop_active);
  EXPECT_EQ(health.error_class, TransportErrorClass::Warning);
}

TEST(SocketCanBackendTest, OpenFailsSafelyForMissingInterface)
{
  SocketCanBackend backend(SocketCanConfig{"__missing_can_interface__"});

  const auto result = backend.open();
  EXPECT_FALSE(result.ok);
  EXPECT_EQ(result.frames_sent, 0U);
  EXPECT_FALSE(backend.health().connected);
  EXPECT_FALSE(backend.health().writable);
  EXPECT_TRUE(backend.health().safe_stop_active);
}

TEST(SocketCanBackendTest, GroupValidationFailsBeforeAnyTransmission)
{
  SocketCanBackend backend;
  const std::array<CanFrame, 2> invalid_group{{valid_command_frame(0x120U), valid_command_frame(
      0x121U)}};
  auto group = invalid_group;
  group[1].data.pop_back();

  const auto result = backend.send_control_group(group, 0.0);
  EXPECT_FALSE(result.ok);
  EXPECT_EQ(result.frames_sent, 0U);
  EXPECT_EQ(result.reason, "transport_closed");
  EXPECT_TRUE(backend.health().safe_stop_active);
  EXPECT_EQ(backend.health().error_class, TransportErrorClass::TxFailure);
}

TEST(SocketCanBackendTest, ReconnectToMissingInterfaceFailsAndCloseIsIdempotent)
{
  SocketCanBackend backend(SocketCanConfig{"__missing_can_interface__"});
  EXPECT_FALSE(backend.reconnect(0.0).ok);
  backend.close();
  backend.close();
  EXPECT_FALSE(backend.health().connected);
  EXPECT_TRUE(backend.health().safe_stop_active);
}

}  // namespace
