#include "car_hardware_adapter/protocol_codec.hpp"

#include <array>
#include <cstdint>
#include <optional>
#include <vector>

#include <gtest/gtest.h>

using namespace car_hardware_adapter;

namespace
{

std::vector<std::uint8_t> header(std::uint16_t sequence, std::uint8_t flags)
{
  return {
    0x01U,
    static_cast<std::uint8_t>(sequence & 0xFFU),
    static_cast<std::uint8_t>((sequence >> 8U) & 0xFFU),
    flags};
}

CanFrame status_frame(
  std::uint16_t sequence, std::uint8_t heartbeat = 1U,
  std::uint16_t applied_sequence = 1U, std::uint16_t fault_code = 0U)
{
  auto data = header(sequence, heartbeat);
  data.push_back(static_cast<std::uint8_t>(applied_sequence & 0xFFU));
  data.push_back(static_cast<std::uint8_t>((applied_sequence >> 8U) & 0xFFU));
  data.push_back(static_cast<std::uint8_t>(fault_code & 0xFFU));
  data.push_back(static_cast<std::uint8_t>((fault_code >> 8U) & 0xFFU));
  return CanFrame{0x180U, false, false, data};
}

CanFrame health_frame(
  std::uint16_t sequence, std::uint8_t flags = 0x0FU,
  std::uint32_t device_time_ms = 1000U)
{
  auto data = header(sequence, flags);
  data.push_back(static_cast<std::uint8_t>(device_time_ms & 0xFFU));
  data.push_back(static_cast<std::uint8_t>((device_time_ms >> 8U) & 0xFFU));
  data.push_back(static_cast<std::uint8_t>((device_time_ms >> 16U) & 0xFFU));
  data.push_back(static_cast<std::uint8_t>((device_time_ms >> 24U) & 0xFFU));
  return CanFrame{0x181U, false, false, data};
}

CanFrame velocity_frame(
  std::uint16_t sequence, std::uint8_t flags = 0x03U,
  std::int16_t left_mmps = 900, std::int16_t right_mmps = 1100)
{
  auto data = header(sequence, flags);
  const auto left = static_cast<std::uint16_t>(left_mmps);
  const auto right = static_cast<std::uint16_t>(right_mmps);
  data.push_back(static_cast<std::uint8_t>(left & 0xFFU));
  data.push_back(static_cast<std::uint8_t>((left >> 8U) & 0xFFU));
  data.push_back(static_cast<std::uint8_t>(right & 0xFFU));
  data.push_back(static_cast<std::uint8_t>((right >> 8U) & 0xFFU));
  return CanFrame{0x182U, false, false, data};
}

CanFrame position_frame(
  std::uint32_t id, std::uint16_t sequence, std::uint8_t flags,
  std::int32_t position_mrad)
{
  const auto value = static_cast<std::uint32_t>(position_mrad);
  auto data = header(sequence, flags);
  data.push_back(static_cast<std::uint8_t>(value & 0xFFU));
  data.push_back(static_cast<std::uint8_t>((value >> 8U) & 0xFFU));
  data.push_back(static_cast<std::uint8_t>((value >> 16U) & 0xFFU));
  data.push_back(static_cast<std::uint8_t>((value >> 24U) & 0xFFU));
  return CanFrame{id, false, false, data};
}

CanFrame diagnostics_frame(
  std::uint16_t sequence, std::uint8_t diagnostic_flags = 0x06U,
  std::uint8_t safety_state = 0x04U, std::uint8_t safety_action = 0x01U,
  std::uint8_t command_age_10ms = 0U, std::uint8_t can_error_class = 0U)
{
  auto data = header(sequence, diagnostic_flags);
  data.push_back(safety_state);
  data.push_back(safety_action);
  data.push_back(command_age_10ms);
  data.push_back(can_error_class);
  return CanFrame{0x186U, false, false, data};
}

std::array<CanFrame, 6> complete_frames(
  std::uint16_t sequence, std::uint32_t device_time_ms,
  std::int32_t left_position_mrad = 0, std::int32_t right_position_mrad = 0)
{
  return {{
    status_frame(sequence, static_cast<std::uint8_t>(sequence & 0xFFU), sequence),
    health_frame(sequence, 0x0FU, device_time_ms),
    velocity_frame(sequence),
    position_frame(0x183U, sequence, 0x01U, left_position_mrad),
    position_frame(0x184U, sequence, 0x01U, right_position_mrad),
    diagnostics_frame(sequence)
  }};
}

std::optional<FeedbackObservation> ingest_complete(
  ProtocolCodec & codec, std::uint16_t sequence, std::uint32_t device_time_ms,
  std::int32_t left_position_mrad = 0, std::int32_t right_position_mrad = 0)
{
  std::optional<FeedbackObservation> result;
  for (const auto & frame : complete_frames(
      sequence, device_time_ms, left_position_mrad, right_position_mrad))
  {
    const auto decoded = codec.ingest_feedback_frame(frame);
    EXPECT_EQ(decoded.status, DecodeStatus::Ok);
    if (decoded.observation.has_value()) {
      result = decoded.observation;
    }
  }
  return result;
}

}  // namespace

TEST(CanV1CodecTest, EncodesControlGroupUsingV1Payload)
{
  ProtocolCodec codec;
  const auto result = codec.encode_control_group(
    ControlCommand{0x1234U, ControlMode::Velocity, false, false, false,
      0.250, 0.900, 1.100});
  ASSERT_TRUE(result.ok);
  EXPECT_EQ(
    result.frames[0],
    (CanFrame{0x120U, false, false, {0x01U, 0x34U, 0x12U, 0x01U, 0xFAU, 0x00U, 0x00U, 0x00U}}));
  EXPECT_EQ(
    result.frames[1],
    (CanFrame{0x121U, false, false, {0x01U, 0x34U, 0x12U, 0x01U, 0x84U, 0x03U, 0x4CU, 0x04U}}));
}

TEST(CanV1CodecTest, RejectsInvalidControlValue)
{
  auto command = ControlCommand{};
  command.mode = ControlMode::Velocity;
  command.equivalent_steering_rad = 1e100;
  const auto result = ProtocolCodec{}.encode_control_group(command);
  EXPECT_FALSE(result.ok);
  EXPECT_EQ(result.reason, "control_value_out_of_range");
}

TEST(CanV1CodecTest, CompleteSnapshotUsesRawMradPositionsAndAllSixFrames)
{
  ProtocolCodec codec;
  const auto frames = complete_frames(0x0102U, 1234U, 3142, 2718);
  for (std::size_t i = 0U; i + 1U < frames.size(); ++i) {
    const auto decoded = codec.ingest_feedback_frame(frames[i]);
    ASSERT_EQ(decoded.status, DecodeStatus::Ok);
    EXPECT_FALSE(decoded.observation.has_value());
  }
  const auto decoded = codec.ingest_feedback_frame(frames.back());
  ASSERT_TRUE(decoded.observation.has_value());
  EXPECT_EQ(decoded.observation->sequence, 0x0102U);
  EXPECT_EQ(decoded.observation->device_time_ms, 1234U);
  EXPECT_DOUBLE_EQ(decoded.observation->left_position_mrad, 3142.0);
  EXPECT_DOUBLE_EQ(decoded.observation->right_position_mrad, 2718.0);
  EXPECT_TRUE(decoded.observation->left_position_valid);
  EXPECT_TRUE(decoded.observation->right_position_valid);
}

TEST(CanV1CodecTest, IgnoresOptionalMaintenanceFeedbackWithoutCountingDrop)
{
  ProtocolCodec codec;
  const CanFrame pid_gains{
    can_v1::kFbPidGains, false, false,
    {0x01U, 0x00U, 0x00U, 0x00U, 0x00U, 0x00U, 0x00U, 0x00U}};
  const CanFrame control_output{
    can_v1::kFbControlOutput, false, false,
    {0x01U, 0x25U, 0x02U, 0x03U, 0x00U, 0x00U, 0x00U, 0x00U}};

  const auto pid_result = codec.ingest_feedback_frame(pid_gains);
  const auto output_result = codec.ingest_feedback_frame(control_output);
  EXPECT_EQ(pid_result.status, DecodeStatus::Ok);
  EXPECT_FALSE(pid_result.observation.has_value());
  EXPECT_EQ(output_result.status, DecodeStatus::Ok);
  EXPECT_FALSE(output_result.observation.has_value());
}

TEST(CanV1CodecTest, ArbitraryFrameOrderStillRequiresOneCompleteSequence)
{
  ProtocolCodec codec;
  const auto frames = complete_frames(7U, 100U, 10, 20);
  const std::array<std::size_t, 6> order{{5U, 3U, 1U, 4U, 0U, 2U}};
  for (std::size_t i = 0U; i + 1U < order.size(); ++i) {
    EXPECT_FALSE(codec.ingest_feedback_frame(frames[order[i]]).observation.has_value());
  }
  const auto decoded = codec.ingest_feedback_frame(frames[order.back()]);
  ASSERT_TRUE(decoded.observation.has_value());
  EXPECT_TRUE(decoded.observation->diagnostics_present);
  EXPECT_TRUE(decoded.observation->status_present);
  EXPECT_TRUE(decoded.observation->health_present);
  EXPECT_TRUE(decoded.observation->velocity_present);
}

TEST(CanV1CodecTest, DuplicateFrameDoesNotRepublishOrOverwriteSnapshot)
{
  ProtocolCodec codec;
  const auto frames = complete_frames(8U, 100U, 10, 20);
  ASSERT_TRUE(ingest_complete(codec, 8U, 100U, 10, 20).has_value());
  const auto duplicate = codec.ingest_feedback_frame(
    position_frame(0x183U, 8U, 0x01U, 9999));
  EXPECT_EQ(duplicate.status, DecodeStatus::Ok);
  EXPECT_FALSE(duplicate.observation.has_value());
}

TEST(CanV1CodecTest, OutOfOrderSequenceIsRejected)
{
  ProtocolCodec codec;
  ASSERT_TRUE(ingest_complete(codec, 10U, 100U).has_value());
  const auto old = codec.ingest_feedback_frame(status_frame(9U));
  EXPECT_EQ(old.status, DecodeStatus::SequenceMismatch);
  EXPECT_FALSE(old.observation.has_value());
}

TEST(CanV1CodecTest, ForwardSequenceGapStartsACompleteNewSnapshot)
{
  ProtocolCodec codec;
  ASSERT_TRUE(ingest_complete(codec, 10U, 100U, 100, 100).has_value());
  const auto frames = complete_frames(12U, 120U, 300, 300);
  for (std::size_t i = 0U; i + 1U < frames.size(); ++i) {
    EXPECT_FALSE(codec.ingest_feedback_frame(frames[i]).observation.has_value());
  }
  const auto decoded = codec.ingest_feedback_frame(frames.back());
  ASSERT_TRUE(decoded.observation.has_value());
  EXPECT_EQ(decoded.observation->sequence, 12U);
  EXPECT_DOUBLE_EQ(decoded.observation->left_position_mrad, 300.0);
}

TEST(CanV1CodecTest, SequenceWrapIsAccepted)
{
  ProtocolCodec codec;
  ASSERT_TRUE(ingest_complete(codec, 0xFFFFU, 100U).has_value());
  ASSERT_TRUE(ingest_complete(codec, 0x0000U, 120U).has_value());
}

TEST(CanV1CodecTest, DeviceTimeRegressionIsNotRejectedByCodec)
{
  ProtocolCodec codec;
  ASSERT_TRUE(ingest_complete(codec, 20U, 1000U).has_value());
  ASSERT_TRUE(ingest_complete(codec, 21U, 900U).has_value());
}

TEST(CanV1CodecTest, DiagnosticsDecodeSafetyGate)
{
  ProtocolCodec codec;
  const auto frames = complete_frames(30U, 100U);
  auto invalid = frames;
  invalid[5] = diagnostics_frame(30U, 0x01U, 0x01U, 0x02U, 9U, 3U);
  for (std::size_t i = 0U; i + 1U < invalid.size(); ++i) {
    EXPECT_FALSE(codec.ingest_feedback_frame(invalid[i]).observation.has_value());
  }
  const auto decoded = codec.ingest_feedback_frame(invalid.back());
  ASSERT_TRUE(decoded.observation.has_value());
  EXPECT_TRUE(decoded.observation->calibration_required);
  EXPECT_FALSE(decoded.observation->drive_allowed);
  EXPECT_EQ(decoded.observation->can_error_class, 3U);
}

TEST(CanV1CodecTest, RejectsInvalidFrameShapeAndFlags)
{
  ProtocolCodec codec;
  EXPECT_EQ(
    codec.ingest_feedback_frame(CanFrame{0x186U, false, false, {}}).status,
    DecodeStatus::InvalidDlc);
  auto bad = health_frame(1U, 0x10U, 100U);
  EXPECT_EQ(codec.ingest_feedback_frame(bad).status, DecodeStatus::InvalidFlags);
  auto remote = status_frame(1U);
  remote.rtr = true;
  EXPECT_EQ(codec.ingest_feedback_frame(remote).status, DecodeStatus::RemoteFrame);
}

TEST(CanV1CodecTest, UnknownAndReservedFramesAreRejected)
{
  ProtocolCodec codec;
  EXPECT_EQ(
    codec.ingest_feedback_frame(
      CanFrame{0x999U, false, false, std::vector<std::uint8_t>(
          8U,
          0U)}).status,
    DecodeStatus::UnknownId);
  auto extended = status_frame(1U);
  extended.extended = true;
  EXPECT_EQ(codec.ingest_feedback_frame(extended).status, DecodeStatus::ExtendedId);
}
