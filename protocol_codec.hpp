#pragma once

#include <array>
#include <cstdint>
#include <optional>
#include <string>
#include <vector>

namespace can_v1
{

constexpr std::uint32_t kCmdSteering = 0x120U;
constexpr std::uint32_t kCmdRearWheels = 0x121U;
constexpr std::uint32_t kFbStatus = 0x180U;
constexpr std::uint32_t kFbHealth = 0x181U;
constexpr std::uint32_t kFbRearVelocity = 0x182U;
constexpr std::uint32_t kFbRearLeftPosition = 0x183U;
constexpr std::uint32_t kFbRearRightPosition = 0x184U;
constexpr std::uint32_t kFbDiagnostics = 0x186U;
constexpr std::uint32_t kFbPidGains = 0x187U;
constexpr std::uint32_t kFbControlOutput = 0x188U;
constexpr std::uint8_t kProtocolVersion = 0x01U;

}  // namespace can_v1

namespace car_hardware_adapter
{

struct CanFrame
{
  std::uint32_t id{0};
  bool extended{false};
  bool rtr{false};
  std::vector<std::uint8_t> data;

  bool operator==(const CanFrame & other) const
  {
    return id == other.id && extended == other.extended && rtr == other.rtr &&
           data == other.data;
  }
};

enum class ControlMode : std::uint8_t
{
  Stop = 0,
  Velocity = 1,
};

struct ControlCommand
{
  std::uint16_t sequence{0};
  ControlMode mode{ControlMode::Stop};
  bool estop_active{false};
  bool safe_stop_active{false};
  bool reset_fault_request{false};
  double equivalent_steering_rad{0.0};
  double rear_left_velocity_mps{0.0};
  double rear_right_velocity_mps{0.0};
};

struct EncodeControlResult
{
  bool ok{false};
  std::array<CanFrame, 2> frames;
  std::string reason;
};

enum class DecodeStatus
{
  Ok,
  UnknownId,
  ExtendedId,
  RemoteFrame,
  InvalidDlc,
  UnknownVersion,
  ReservedNonZero,
  InvalidFlags,
  InvalidEnum,
  SequenceMismatch,
  DeviceTimeRegressed
};

struct FeedbackObservation
{
  std::uint16_t sequence{0};
  bool velocity_present{false};
  bool status_present{false};
  bool health_present{false};
  bool left_position_valid{false};
  bool right_position_valid{false};
  std::uint8_t heartbeat_counter{0};
  std::uint16_t applied_command_sequence{0};
  std::uint32_t device_time_ms{0};
  double left_velocity_mps{0.0};
  double right_velocity_mps{0.0};
  double left_position_mrad{0.0};
  double right_position_mrad{0.0};
  std::uint8_t status_flags{0};
  std::uint16_t fault_code{0};
  bool left_velocity_valid{false};
  bool right_velocity_valid{false};
  bool diagnostics_present{false};
  bool diagnostics_values_known{false};
  bool calibration_required{false};
  bool drive_allowed{false};
  bool command_fresh{false};
  std::uint8_t diagnostic_flags{0};
  std::uint8_t safety_state{0xFFU};
  std::uint8_t safety_action{0xFFU};
  std::uint8_t command_age_10ms{0xFFU};
  std::uint8_t can_error_class{0xFFU};
};

struct IngestResult
{
  DecodeStatus status{DecodeStatus::UnknownId};
  std::optional<FeedbackObservation> observation;
};

class ProtocolCodec
{
public:
  EncodeControlResult encode_control_group(const ControlCommand & command) const;
  IngestResult ingest_feedback_frame(const CanFrame & frame);
  void reset();

private:
  struct PartialFeedback
  {
    FeedbackObservation value;
    std::array<bool, 6> frame_received{};
  };

  std::optional<PartialFeedback> partial_;
};

struct FaultInfo
{
  std::string fault_reason;
  std::string recovery_hint;
};

FaultInfo describe_fault(std::uint16_t fault_code);

}  // namespace car_hardware_adapter
