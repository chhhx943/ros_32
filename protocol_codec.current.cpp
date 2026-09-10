#include "car_hardware_adapter/protocol_codec.hpp"

#include <cmath>
#include <cstddef>
#include <limits>

namespace car_hardware_adapter
{

namespace
{

constexpr double kScale = 1000.0;

// CAN_PROTOCOL V1 diagnostic_flags: all eight bits are assigned by firmware.
constexpr std::uint8_t kDiagnosticCalibrationRequired = 0x01U;  // bit 0
constexpr std::uint8_t kDiagnosticDriveAllowed = 0x02U;         // bit 1
constexpr std::uint8_t kDiagnosticCommandFresh = 0x04U;          // bit 2
constexpr std::uint8_t kDiagnosticLeftEncoderInvalid = 0x08U;   // bit 3
constexpr std::uint8_t kDiagnosticRightEncoderInvalid = 0x10U;  // bit 4
constexpr std::uint8_t kDiagnosticControlOverrun = 0x20U;        // bit 5
constexpr std::uint8_t kDiagnosticMotorStall = 0x40U;            // bit 6
constexpr std::uint8_t kDiagnosticCanError = 0x80U;              // bit 7
// The MCU watchdog window is 100 ms; ages 0..9 are the current 10 ms ticks.
constexpr std::uint8_t kCommandAgeFreshMax10ms = 9U;
constexpr std::uint8_t kHealthReservedStatusFlag = 0x10U;       // bit 4 is reserved

constexpr std::uint8_t kDiagnosticHighRiskFlags =
  kDiagnosticCalibrationRequired | kDiagnosticLeftEncoderInvalid |
  kDiagnosticRightEncoderInvalid | kDiagnosticControlOverrun | kDiagnosticMotorStall |
  kDiagnosticCanError;

// CAN_PROTOCOL V1 safety_state enum.
constexpr std::uint8_t kSafetyStateCalibrationRequired = 1U;
constexpr std::uint8_t kSafetyStateCalibration = 2U;
constexpr std::uint8_t kSafetyStateDrive = 4U;
constexpr std::uint8_t kKnownSafetyStateMax = 7U;

// CAN_PROTOCOL V1 safety_action enum.
constexpr std::uint8_t kSafetyActionDrive = 1U;
constexpr std::uint8_t kSafetyActionCalibration = 2U;
constexpr std::uint8_t kKnownSafetyActionMax = 3U;

void put_u16_le(std::vector<std::uint8_t> & data, std::size_t offset, std::uint16_t value)
{
  data[offset] = static_cast<std::uint8_t>(value & 0xFFU);
  data[offset + 1U] = static_cast<std::uint8_t>((value >> 8U) & 0xFFU);
}

void put_i16_le(std::vector<std::uint8_t> & data, std::size_t offset, std::int16_t value)
{
  put_u16_le(data, offset, static_cast<std::uint16_t>(value));
}

std::uint16_t read_u16_le(const std::vector<std::uint8_t> & data, std::size_t offset)
{
  return static_cast<std::uint16_t>(
    static_cast<std::uint16_t>(data[offset]) |
    (static_cast<std::uint16_t>(data[offset + 1U]) << 8U));
}

std::int16_t read_i16_le(const std::vector<std::uint8_t> & data, std::size_t offset)
{
  return static_cast<std::int16_t>(read_u16_le(data, offset));
}

std::uint32_t read_u32_le(const std::vector<std::uint8_t> & data, std::size_t offset)
{
  return static_cast<std::uint32_t>(
    static_cast<std::uint32_t>(data[offset]) |
    (static_cast<std::uint32_t>(data[offset + 1U]) << 8U) |
    (static_cast<std::uint32_t>(data[offset + 2U]) << 16U) |
    (static_cast<std::uint32_t>(data[offset + 3U]) << 24U));
}

std::int32_t read_i32_le(const std::vector<std::uint8_t> & data, std::size_t offset)
{
  return static_cast<std::int32_t>(read_u32_le(data, offset));
}

bool to_i16_scaled(double value, std::int16_t & out)
{
  if (!std::isfinite(value)) {
    return false;
  }

  // Keep value * kScale in a small, defined range before calling lround.
  constexpr double kHalfUnit = 0.5 / kScale;
  const auto min_value = static_cast<double>(std::numeric_limits<std::int16_t>::min()) / kScale;
  const auto max_value = static_cast<double>(std::numeric_limits<std::int16_t>::max()) / kScale;
  if (value < min_value - kHalfUnit || value > max_value + kHalfUnit) {
    return false;
  }

  const auto scaled = std::lround(value * kScale);
  if (
    scaled < std::numeric_limits<std::int16_t>::min() ||
    scaled > std::numeric_limits<std::int16_t>::max())
  {
    return false;
  }

  out = static_cast<std::int16_t>(scaled);
  return true;
}

std::uint8_t mode_flags(const ControlCommand & command)
{
  std::uint8_t flags = static_cast<std::uint8_t>(command.mode);
  if (command.estop_active) {
    flags = static_cast<std::uint8_t>(flags | 0x04U);
  }
  if (command.safe_stop_active) {
    flags = static_cast<std::uint8_t>(flags | 0x08U);
  }
  if (command.reset_fault_request) {
    flags = static_cast<std::uint8_t>(flags | 0x10U);
  }
  return flags;
}

bool all_targets_zero(const ControlCommand & command)
{
  return command.equivalent_steering_rad == 0.0 && command.rear_left_velocity_mps == 0.0 &&
         command.rear_right_velocity_mps == 0.0;
}

bool is_feedback_id(std::uint32_t id)
{
  return
    (id >= can_v1::kFbStatus && id <= can_v1::kFbRearRightPosition) ||
    id == can_v1::kFbDiagnostics;
}

bool is_optional_feedback_id(std::uint32_t id)
{
  return id == can_v1::kFbPidGains || id == can_v1::kFbControlOutput;
}

std::optional<std::size_t> feedback_frame_slot(std::uint32_t id)
{
  switch (id) {
    case can_v1::kFbStatus:
      return std::optional<std::size_t>(0U);
    case can_v1::kFbHealth:
      return std::optional<std::size_t>(1U);
    case can_v1::kFbRearVelocity:
      return std::optional<std::size_t>(2U);
    case can_v1::kFbRearLeftPosition:
      return std::optional<std::size_t>(3U);
    case can_v1::kFbRearRightPosition:
      return std::optional<std::size_t>(4U);
    case can_v1::kFbDiagnostics:
      return std::optional<std::size_t>(5U);
    default:
      return std::nullopt;
  }
}

bool diagnostics_are_known(
  std::uint8_t safety_state, std::uint8_t safety_action, std::uint8_t can_error_class)
{
  return
    safety_state <= kKnownSafetyStateMax && safety_action <= kKnownSafetyActionMax &&
    can_error_class <= 4U;
}

bool diagnostics_allow_drive(const FeedbackObservation & value)
{
  if (!value.diagnostics_values_known || value.calibration_required || !value.command_fresh) {
    return false;
  }

  if (value.safety_state != kSafetyStateDrive || value.safety_action != kSafetyActionDrive) {
    return false;
  }

  if ((value.diagnostic_flags & kDiagnosticDriveAllowed) == 0U) {
    return false;
  }

  if ((value.diagnostic_flags & kDiagnosticCommandFresh) == 0U) {
    return false;
  }

  if ((value.diagnostic_flags & kDiagnosticHighRiskFlags) != 0U) {
    return false;
  }

  if (value.can_error_class != 0U) {
    return false;
  }

  return true;
}

}  // namespace

EncodeControlResult ProtocolCodec::encode_control_group(const ControlCommand & command) const
{
  if (command.mode != ControlMode::Stop && command.mode != ControlMode::Velocity) {
    return EncodeControlResult{false, {}, "unknown_control_mode"};
  }

  if (
    !std::isfinite(command.equivalent_steering_rad) ||
    !std::isfinite(command.rear_left_velocity_mps) ||
    !std::isfinite(command.rear_right_velocity_mps))
  {
    return EncodeControlResult{false, {}, "control_value_not_finite"};
  }

  if (
    command.mode == ControlMode::Stop &&
    (command.rear_left_velocity_mps != 0.0 || command.rear_right_velocity_mps != 0.0))
  {
    return EncodeControlResult{false, {}, "stop_requires_zero_rear_targets"};
  }

  if (
    command.reset_fault_request &&
    (command.mode != ControlMode::Stop || command.estop_active || !all_targets_zero(command)))
  {
    return EncodeControlResult{false, {}, "invalid_reset_fault_request"};
  }

  std::int16_t steering_mrad = 0;
  std::int16_t rear_left_mmps = 0;
  std::int16_t rear_right_mmps = 0;
  if (
    !to_i16_scaled(command.equivalent_steering_rad, steering_mrad) ||
    !to_i16_scaled(command.rear_left_velocity_mps, rear_left_mmps) ||
    !to_i16_scaled(command.rear_right_velocity_mps, rear_right_mmps))
  {
    return EncodeControlResult{false, {}, "control_value_out_of_range"};
  }

  const auto flags = mode_flags(command);
  EncodeControlResult result;
  result.ok = true;
  result.frames[0] = CanFrame{
    can_v1::kCmdSteering, false, false, std::vector<std::uint8_t>(8U, 0U)};
  result.frames[1] = CanFrame{
    can_v1::kCmdRearWheels, false, false, std::vector<std::uint8_t>(8U, 0U)};

  for (auto & frame : result.frames) {
    frame.data[0] = can_v1::kProtocolVersion;
    put_u16_le(frame.data, 1U, command.sequence);
    frame.data[3] = flags;
  }

  put_i16_le(result.frames[0].data, 4U, steering_mrad);
  put_i16_le(result.frames[1].data, 4U, rear_left_mmps);
  put_i16_le(result.frames[1].data, 6U, rear_right_mmps);
  return result;
}

IngestResult ProtocolCodec::ingest_feedback_frame(const CanFrame & frame)
{
  if (frame.extended) {
    return IngestResult{DecodeStatus::ExtendedId, std::nullopt};
  }
  if (frame.rtr) {
    return IngestResult{DecodeStatus::RemoteFrame, std::nullopt};
  }
  if (!is_feedback_id(frame.id)) {
    if (!is_optional_feedback_id(frame.id)) {
      return IngestResult{DecodeStatus::UnknownId, std::nullopt};
    }
    if (frame.data.size() != 8U) {
      return IngestResult{DecodeStatus::InvalidDlc, std::nullopt};
    }
    if (frame.data[0] != can_v1::kProtocolVersion) {
      return IngestResult{DecodeStatus::UnknownVersion, std::nullopt};
    }
    // Maintenance/diagnostic extensions are valid bus traffic, but they are
    // not part of the six-frame odometry snapshot and must not disturb the
    // snapshot assembler or inflate the transport drop counter.
    return IngestResult{DecodeStatus::Ok, std::nullopt};
  }
  if (frame.data.size() != 8U) {
    return IngestResult{DecodeStatus::InvalidDlc, std::nullopt};
  }
  if (frame.data[0] != can_v1::kProtocolVersion) {
    return IngestResult{DecodeStatus::UnknownVersion, std::nullopt};
  }
  if (frame.id == can_v1::kFbHealth && (frame.data[3] & kHealthReservedStatusFlag) != 0U) {
    return IngestResult{DecodeStatus::InvalidFlags, std::nullopt};
  }
  if (frame.id == can_v1::kFbRearVelocity && (frame.data[3] & 0xFCU) != 0U) {
    return IngestResult{DecodeStatus::InvalidFlags, std::nullopt};
  }
  if (
    (frame.id == can_v1::kFbRearLeftPosition || frame.id == can_v1::kFbRearRightPosition) &&
    (frame.data[3] & 0xFEU) != 0U)
  {
    return IngestResult{DecodeStatus::InvalidFlags, std::nullopt};
  }

  const auto sequence = read_u16_le(frame.data, 1U);
  const auto frame_slot = feedback_frame_slot(frame.id);
  if (!frame_slot.has_value()) {
    return IngestResult{DecodeStatus::UnknownId, std::nullopt};
  }

  bool starts_new_partial = !partial_.has_value();
  if (!starts_new_partial) {
    const auto current = partial_->value.sequence;
    const auto forward = static_cast<std::uint16_t>(sequence - current);
    if (forward == 0U) {
      if (partial_->frame_received[*frame_slot]) {
        return IngestResult{DecodeStatus::Ok, std::nullopt};
      }
    } else if (forward <= 0x7FFFU) {
      starts_new_partial = true;
    } else {
      return IngestResult{DecodeStatus::SequenceMismatch, std::nullopt};
    }
  }

  if (starts_new_partial) {
    partial_ = PartialFeedback{};
    partial_->value.sequence = sequence;
  }

  partial_->frame_received[*frame_slot] = true;
  auto & value = partial_->value;
  switch (frame.id) {
    case can_v1::kFbStatus:
      value.status_present = true;
      value.heartbeat_counter = frame.data[3];
      value.applied_command_sequence = read_u16_le(frame.data, 4U);
      value.fault_code = read_u16_le(frame.data, 6U);
      break;

    case can_v1::kFbHealth:
      value.health_present = true;
      value.status_flags = frame.data[3];
      value.device_time_ms = read_u32_le(frame.data, 4U);
      break;

    case can_v1::kFbRearVelocity:
      value.velocity_present = true;
      value.left_velocity_valid = (frame.data[3] & 0x01U) != 0U;
      value.right_velocity_valid = (frame.data[3] & 0x02U) != 0U;
      value.left_velocity_mps = static_cast<double>(read_i16_le(frame.data, 4U)) / kScale;
      value.right_velocity_mps = static_cast<double>(read_i16_le(frame.data, 6U)) / kScale;
      break;

    case can_v1::kFbRearLeftPosition:
      value.left_position_valid = (frame.data[3] & 0x01U) != 0U;
      value.left_position_mrad = static_cast<double>(read_i32_le(frame.data, 4U));
      break;

    case can_v1::kFbRearRightPosition:
      value.right_position_valid = (frame.data[3] & 0x01U) != 0U;
      value.right_position_mrad = static_cast<double>(read_i32_le(frame.data, 4U));
      break;

    case can_v1::kFbDiagnostics:
      value.diagnostics_present = true;
      value.diagnostic_flags = frame.data[3];
      value.safety_state = frame.data[4];
      value.safety_action = frame.data[5];
      value.command_age_10ms = frame.data[6];
      value.can_error_class = frame.data[7];
      value.diagnostics_values_known = diagnostics_are_known(
        value.safety_state, value.safety_action, value.can_error_class);
      value.calibration_required =
        (value.diagnostic_flags & kDiagnosticCalibrationRequired) != 0U ||
        value.safety_state == kSafetyStateCalibrationRequired ||
        value.safety_state == kSafetyStateCalibration ||
        value.safety_action == kSafetyActionCalibration;
      value.command_fresh =
        (value.diagnostic_flags & kDiagnosticCommandFresh) != 0U &&
        value.command_age_10ms <= kCommandAgeFreshMax10ms;
      value.drive_allowed = diagnostics_allow_drive(value);
      break;

    default:
      return IngestResult{DecodeStatus::UnknownId, std::nullopt};
  }

  bool complete = true;
  for (const auto received : partial_->frame_received) {
    complete = complete && received;
  }
  if (!complete) {
    return IngestResult{DecodeStatus::Ok, std::nullopt};
  }
  return IngestResult{DecodeStatus::Ok, partial_->value};
}

void ProtocolCodec::reset()
{
  partial_.reset();
}

FaultInfo describe_fault(std::uint16_t fault_code)
{
  switch (fault_code) {
    case 0x0000:
      return FaultInfo{"none", "none"};
    case 0x0001:
      return FaultInfo{"estop_active", "release_estop_and_request_focus_again"};
    case 0x0002:
      return FaultInfo{"rear_encoder_fault", "stop_and_check_rear_encoder_wiring"};
    case 0x0003:
      return FaultInfo{"steering_command_rejected", "stop_and_check_steering_calibration"};
    case 0x0004:
      return FaultInfo{"command_timeout", "send_fresh_zero_command_then_rearm"};
    case 0x0005:
      return FaultInfo{"command_group_incomplete", "check_can_loss_and_command_pairing"};
    case 0x0006:
      return FaultInfo{"command_group_inconsistent", "check_host_codec_sequence_and_flags"};
    case 0x0007:
      return FaultInfo{"command_range_invalid", "check_vehicle_profile_limits"};
    case 0x0008:
      return FaultInfo{"mcu_watchdog_reset", "inspect_mcu_reset_reason_and_power"};
    case 0x0009:
      return FaultInfo{"motor_driver_fault", "stop_and_check_motor_driver"};
    case 0x000A:
      return FaultInfo{"calibration_invalid", "fix_mcu_steering_or_encoder_calibration"};
    case 0xFFFF:
      return FaultInfo{"unknown_device_fault", "stop_and_check_device_fault_register"};
    default:
      return FaultInfo{"unknown_device_fault", "stop_and_check_device_fault_register"};
  }
}

}  // namespace car_hardware_adapter
