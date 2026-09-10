#pragma once

#include "car_hardware_adapter/protocol_codec.hpp"
#include "car_hardware_adapter/chassis_backend.hpp"

#include <cstdint>
#include <optional>
#include <string>

namespace car_hardware_adapter
{

bool is_ros_time_ready(std::int64_t nanoseconds) noexcept;

enum class AdapterState
{
  Init,
  Connecting,
  Ready,
  Active,
  Degraded,
  Fault
};

struct AdapterConfig
{
  double wheelbase_m;
  double rear_track_m;
  double max_equivalent_steering_angle_rad;
  double max_rear_wheel_speed_mps;
  double command_timeout_sec;
  double feedback_timeout_sec;
  double heartbeat_timeout_sec;
  double max_odometry_step_sec;
  bool allow_motion{false};
  double wheel_radius_m{0.03325};
  int left_position_sign{1};
  int right_position_sign{1};
  double max_position_delta_m{0.20};
};

struct AdapterStatus
{
  AdapterState state{AdapterState::Init};
  bool safe_stop_active{true};
  bool feedback_valid{false};
  bool heartbeat_ok{false};
  bool fault_latched{false};
  std::uint16_t fault_code{0};
  std::uint64_t feedback_sequence_gap_count{0};
  std::uint64_t odometry_rebase_count{0};
  bool odometry_baseline_valid{false};
};

struct CommandDecision
{
  bool accepted{false};
  std::string reason;
  ControlCommand command;
  double requested_speed_mps{0.0};
  double applied_speed_mps{0.0};
  double requested_steering_rad{0.0};
  double applied_steering_rad{0.0};
  double wheel_speed_scale{1.0};
  bool steering_limited{false};
  bool wheel_speed_limited{false};
};

struct OdometryState
{
  bool valid{false};
  double x_m{0.0};
  double y_m{0.0};
  double yaw_rad{0.0};
  double linear_mps{0.0};
  double angular_radps{0.0};
};

class AdapterCore
{
public:
  explicit AdapterCore(AdapterConfig config);

  void on_backend_open(double now_sec);
  void on_backend_closed(const std::string & reason, double now_sec);
  void on_backend_health(const BackendHealth & health, double now_sec);
  void on_feedback(
    const FeedbackObservation & observation, double now_sec,
    std::optional<double> odometry_now_sec = std::nullopt);
  CommandDecision on_twist(double linear_x, double angular_z, double now_sec);
  ControlCommand tick(double now_sec);
  const AdapterStatus & status() const;
  const OdometryState & odometry() const;

private:
  ControlCommand make_stop_command(bool safe_stop_active);
  void rebase_odometry(const FeedbackObservation & observation);
  void update_odometry(const FeedbackObservation & observation);
  bool config_valid() const;

  AdapterConfig config_;
  AdapterStatus status_;
  OdometryState odometry_;
  bool backend_open_{false};
  bool has_last_command_{false};
  bool has_last_feedback_{false};
  bool has_heartbeat_{false};
  bool has_odom_sample_{false};
  bool odometry_baseline_valid_{false};
  std::optional<std::uint32_t> last_device_time_ms_;
  double last_left_position_mrad_{0.0};
  double last_right_position_mrad_{0.0};
  bool keepalive_pending_{false};
  bool keepalive_acknowledged_{false};
  bool read_only_transport_degraded_{false};
  bool safety_latched_{false};
  double last_command_sec_{0.0};
  double last_feedback_sec_{0.0};
  double last_heartbeat_change_sec_{0.0};
  double last_odom_sec_{0.0};
  std::uint8_t last_heartbeat_counter_{0};
  std::uint16_t next_command_sequence_{0};
  std::optional<std::uint16_t> last_feedback_sequence_;
  ControlCommand current_command_;
};

}  // namespace car_hardware_adapter
