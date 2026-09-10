#include "car_hardware_adapter/adapter_core.hpp"

#include "car_hardware_adapter/chassis_backend.hpp"

#include <algorithm>
#include <cmath>

namespace car_hardware_adapter
{

bool is_ros_time_ready(std::int64_t nanoseconds) noexcept
{
  return nanoseconds > 0;
}

namespace
{

constexpr double kSmall = 1e-9;
constexpr double kHalfPi = 1.5707963267948966;
// Diagnostic bits that invalidate encoder-derived odometry. The calibration
// required bit remains a command-authorization gate, so it is intentionally
// excluded here.
constexpr std::uint8_t kDiagnosticOdomBlockingFlags = 0xF8U;

bool has_left_encoder(const FeedbackObservation & observation)
{
  return !observation.status_present || (observation.status_flags & 0x02U) != 0U;
}

bool has_right_encoder(const FeedbackObservation & observation)
{
  return !observation.status_present || (observation.status_flags & 0x04U) != 0U;
}

bool has_latched_fault(const FeedbackObservation & observation)
{
  return observation.status_present && (observation.status_flags & 0x80U) != 0U;
}

bool has_safe_stop(const FeedbackObservation & observation)
{
  return observation.health_present && (observation.status_flags & 0x40U) != 0U;
}

bool is_read_only_can_warning(const AdapterConfig & config, const BackendHealth & health)
{
  return !config.allow_motion && health.connected && health.heartbeat_ok &&
         (health.error_class == TransportErrorClass::Warning ||
         health.error_class == TransportErrorClass::ErrorPassive);
}

}  // namespace

AdapterCore::AdapterCore(AdapterConfig config)
: config_(config)
{
  // Sequence zero is the explicit initial applied stop.  It is part of the
  // runtime initialization barrier, so construction must not consume it.
  current_command_ = ControlCommand{};
  current_command_.safe_stop_active = true;
}

void AdapterCore::on_backend_health(const BackendHealth & health, double now_sec)
{
  (void) now_sec;
  if (is_read_only_can_warning(config_, health) && !safety_latched_ && !status_.fault_latched) {
    status_.state = AdapterState::Degraded;
    status_.safe_stop_active = true;
    status_.heartbeat_ok = true;
    status_.fault_code = 0U;
    read_only_transport_degraded_ = true;
    has_last_command_ = false;
    current_command_ = make_stop_command(true);
    return;
  }
  if (!health.connected || !health.writable || health.fault_code != 0U ||
    health.error_class != TransportErrorClass::None)
  {
    status_.state = AdapterState::Fault;
    status_.safe_stop_active = true;
    status_.fault_code = health.fault_code;
    status_.fault_latched = true;
    safety_latched_ = true;
    has_last_command_ = false;
    current_command_ = make_stop_command(true);
    return;
  }
  if (!health.heartbeat_ok) {
    status_.state = AdapterState::Fault;
    status_.safe_stop_active = true;
    status_.heartbeat_ok = false;
    status_.fault_latched = true;
    safety_latched_ = true;
    has_last_command_ = false;
    current_command_ = make_stop_command(true);
    return;
  }
  if (health.safe_stop_active) {
    if (safety_latched_ || status_.fault_latched || status_.state == AdapterState::Fault) {
      status_.state = AdapterState::Fault;
      status_.safe_stop_active = true;
      has_last_command_ = false;
      current_command_ = make_stop_command(true);
      return;
    }
    status_.state = AdapterState::Degraded;
    status_.safe_stop_active = true;
    status_.heartbeat_ok = true;
    has_last_command_ = false;
    current_command_ = make_stop_command(true);
    return;
  }
  status_.heartbeat_ok = true;
  if (!safety_latched_ && status_.state != AdapterState::Fault) {
    status_.state = AdapterState::Connecting;
    status_.safe_stop_active = true;
    has_last_command_ = false;
    current_command_ = make_stop_command(true);
  }
}

void AdapterCore::on_backend_open(double now_sec)
{
  (void) now_sec;
  backend_open_ = true;
  has_last_command_ = false;
  has_last_feedback_ = false;
  has_heartbeat_ = false;
  has_odom_sample_ = false;
  odometry_baseline_valid_ = false;
  last_device_time_ms_.reset();
  keepalive_pending_ = false;
  keepalive_acknowledged_ = false;
  read_only_transport_degraded_ = false;
  last_feedback_sequence_.reset();
  safety_latched_ = false;
  next_command_sequence_ = 0;
  odometry_ = OdometryState{};
  status_ = AdapterStatus{};
  status_.state = AdapterState::Connecting;
  status_.safe_stop_active = true;
  current_command_ = ControlCommand{};
  current_command_.safe_stop_active = true;
}

void AdapterCore::on_backend_closed(const std::string & reason, double now_sec)
{
  (void) reason;
  (void) now_sec;
  backend_open_ = false;
  has_last_command_ = false;
  keepalive_pending_ = false;
  keepalive_acknowledged_ = false;
  status_.state = AdapterState::Fault;
  status_.safe_stop_active = true;
  status_.feedback_valid = false;
  status_.heartbeat_ok = false;
  status_.fault_latched = true;
  safety_latched_ = true;
  current_command_ = make_stop_command(true);
}

void AdapterCore::on_feedback(
  const FeedbackObservation & observation, double now_sec,
  std::optional<double> odometry_now_sec)
{
  (void) odometry_now_sec;
  if (!backend_open_) {
    return;
  }

  if (!std::isfinite(now_sec)) {
    odometry_.valid = false;
    odometry_baseline_valid_ = false;
    status_.odometry_baseline_valid = false;
    status_.state = AdapterState::Fault;
    status_.safe_stop_active = true;
    status_.fault_latched = true;
    safety_latched_ = true;
    has_last_command_ = false;
    current_command_ = make_stop_command(true);
    return;
  }

  const auto device_time_regressed = [&]() {
      if (!last_device_time_ms_.has_value() || !observation.health_present) {
        return false;
      }
      const auto delta = static_cast<std::uint32_t>(
        observation.device_time_ms - *last_device_time_ms_);
      return delta > 0x7FFFFFFFU;
    };
  const bool sequence_reinitialized =
    last_feedback_sequence_.has_value() &&
    observation.sequence == 0U &&
    *last_feedback_sequence_ != 0U &&
    *last_feedback_sequence_ < 0xFFF0U;
  const bool epoch_reset = device_time_regressed() || sequence_reinitialized;
  bool duplicate = false;
  if (last_feedback_sequence_.has_value() && !epoch_reset) {
    const auto delta = static_cast<std::uint16_t>(
      observation.sequence - *last_feedback_sequence_);
    if (delta == 0U) {
      duplicate = true;
    } else if (delta > 0x7FFFU) {
      return;
    } else if (delta > 1U) {
      status_.feedback_sequence_gap_count += static_cast<std::uint64_t>(delta - 1U);
    }
  }
  if (duplicate) {
    has_last_feedback_ = true;
    last_feedback_sec_ = now_sec;
    return;
  }

  last_feedback_sequence_ = observation.sequence;
  has_last_feedback_ = true;
  last_feedback_sec_ = now_sec;
  if (observation.status_present) {
    if (!has_heartbeat_) {
      has_heartbeat_ = true;
      last_heartbeat_counter_ = observation.heartbeat_counter;
      last_heartbeat_change_sec_ = now_sec;
    } else {
      const auto delta = static_cast<std::uint8_t>(
        observation.heartbeat_counter - last_heartbeat_counter_);
      if (delta != 0U) {
        last_heartbeat_counter_ = observation.heartbeat_counter;
        last_heartbeat_change_sec_ = now_sec;
      }
    }
  }

  if (!has_last_command_ && next_command_sequence_ == 0U) {
    next_command_sequence_ = 1U;
  }

  status_.feedback_valid =
    observation.velocity_present &&
    observation.left_velocity_valid &&
    observation.right_velocity_valid;
  status_.heartbeat_ok =
    has_heartbeat_ && now_sec - last_heartbeat_change_sec_ <= config_.heartbeat_timeout_sec;
  status_.fault_code = observation.status_present ? observation.fault_code : status_.fault_code;
  status_.fault_latched = status_.fault_latched || has_latched_fault(observation);
  status_.safe_stop_active = has_safe_stop(observation);
  if (!config_.allow_motion) {
    status_.safe_stop_active = true;
  }

  if (
    observation.velocity_present &&
    (!observation.left_velocity_valid || !observation.right_velocity_valid ||
    !std::isfinite(observation.left_velocity_mps) ||
    !std::isfinite(observation.right_velocity_mps)))
  {
    rebase_odometry(observation);
    odometry_.valid = false;
    status_.state = AdapterState::Fault;
    status_.safe_stop_active = true;
    status_.fault_latched = true;
    safety_latched_ = true;
    has_last_command_ = false;
    current_command_ = make_stop_command(true);
    return;
  }

  const bool diagnostics_hard_fault =
    observation.diagnostics_present &&
    (!observation.diagnostics_values_known ||
    observation.can_error_class != 0U ||
    (observation.diagnostic_flags & kDiagnosticOdomBlockingFlags) != 0U);
  if (diagnostics_hard_fault) {
    rebase_odometry(observation);
    status_.state = AdapterState::Fault;
    status_.safe_stop_active = true;
    status_.fault_latched = true;
    safety_latched_ = true;
    has_last_command_ = false;
    current_command_ = make_stop_command(true);
    return;
  }

  // Calibration, command freshness, and DRIVE authorization are safety gates,
  // not reasons to discard valid cumulative encoder feedback. Keep odometry
  // read-only while the gate is closed; on_twist remains fail-closed.
  const bool drive_not_authorized =
    observation.diagnostics_present &&
    (observation.calibration_required || !observation.drive_allowed ||
    !observation.command_fresh || observation.safety_state != 4U ||
    observation.safety_action != 1U);

  const bool encoders_ok =
    has_left_encoder(observation) && has_right_encoder(observation) &&
    observation.left_position_valid && observation.right_position_valid;
  if (!encoders_ok) {
    rebase_odometry(observation);
    status_.state = AdapterState::Fault;
    status_.safe_stop_active = true;
    status_.fault_latched = true;
    status_.fault_code = status_.fault_code == 0U ? 0x0002U : status_.fault_code;
    safety_latched_ = true;
    has_last_command_ = false;
    current_command_ = make_stop_command(true);
    return;
  }

  if (status_.fault_code != 0U || status_.fault_latched || safety_latched_) {
    rebase_odometry(observation);
    status_.state = AdapterState::Fault;
    status_.safe_stop_active = true;
    safety_latched_ = true;
    current_command_ = make_stop_command(true);
    return;
  }

  if (!status_.heartbeat_ok) {
    rebase_odometry(observation);
    status_.state = AdapterState::Fault;
    status_.safe_stop_active = true;
    status_.fault_latched = true;
    safety_latched_ = true;
    has_last_command_ = false;
    current_command_ = make_stop_command(true);
    return;
  }

  if (epoch_reset) {
    rebase_odometry(observation);
    status_.state = AdapterState::Connecting;
    status_.safe_stop_active = true;
    has_last_command_ = false;
    current_command_ = make_stop_command(true);
    return;
  }

  if (drive_not_authorized || has_safe_stop(observation)) {
    // Safe-stop is a command authorization state, not a reason to discard
    // valid encoder feedback. Keep wheel odometry read-only and fail-closed
    // for motion until the safety state clears.
    update_odometry(observation);
    status_.state = AdapterState::Degraded;
    status_.safe_stop_active = true;
    has_last_command_ = false;
    current_command_ = make_stop_command(true);
    return;
  }

  update_odometry(observation);

  if (read_only_transport_degraded_) {
    status_.state = AdapterState::Degraded;
    status_.safe_stop_active = true;
    has_last_command_ = false;
    current_command_ = make_stop_command(true);
    return;
  }

  if (observation.velocity_present) {
    status_.safe_stop_active = !config_.allow_motion;
    status_.state = has_last_command_ ? AdapterState::Active : AdapterState::Ready;
  }
}

CommandDecision AdapterCore::on_twist(double linear_x, double angular_z, double now_sec)
{
  auto stop = current_command_;
  stop.mode = ControlMode::Stop;
  stop.estop_active = false;
  stop.safe_stop_active = true;
  stop.equivalent_steering_rad = 0.0;
  stop.rear_left_velocity_mps = 0.0;
  stop.rear_right_velocity_mps = 0.0;
  if (!std::isfinite(linear_x) || !std::isfinite(angular_z) || !std::isfinite(now_sec)) {
    has_last_command_ = false;
    status_.safe_stop_active = true;
    status_.state = AdapterState::Fault;
    status_.fault_latched = true;
    safety_latched_ = true;
    current_command_ = stop;
    const auto reason = std::isfinite(linear_x) && std::isfinite(angular_z) ?
      "command_time_not_finite" : "command_not_finite";
    return CommandDecision{false, reason, stop};
  }
  if (!backend_open_ || status_.state == AdapterState::Init ||
    status_.state == AdapterState::Fault ||
    status_.state == AdapterState::Degraded)
  {
    return CommandDecision{false, "adapter_not_ready", stop};
  }
  if (!config_.allow_motion) {
    if (std::fabs(linear_x) > kSmall || std::fabs(angular_z) > kSmall) {
      status_.safe_stop_active = true;
      return CommandDecision{false, "motion_disabled", stop};
    }
    has_last_command_ = false;
    current_command_ = stop;
    status_.safe_stop_active = true;
    return CommandDecision{true, "", stop};
  }
  if (!config_valid()) {
    return CommandDecision{false, "vehicle_geometry_unconfigured", stop};
  }
  if (std::fabs(linear_x) <= kSmall && std::fabs(angular_z) > kSmall) {
    return CommandDecision{false, "unsupported_zero_radius_command", stop};
  }
  if (std::fabs(linear_x) <= kSmall && std::fabs(angular_z) <= kSmall) {
    has_last_command_ = false;
    current_command_ = stop;
    status_.state = AdapterState::Ready;
    status_.safe_stop_active = true;
    return CommandDecision{true, "", stop};
  }

  const auto requested_steering = std::atan(config_.wheelbase_m * angular_z / linear_x);
  const auto applied_steering = std::clamp(
    requested_steering,
    -config_.max_equivalent_steering_angle_rad,
    config_.max_equivalent_steering_angle_rad);
  const auto kappa = std::tan(applied_steering) / config_.wheelbase_m;
  const auto raw_left = linear_x * (1.0 - kappa * config_.rear_track_m / 2.0);
  const auto raw_right = linear_x * (1.0 + kappa * config_.rear_track_m / 2.0);
  const auto peak = std::max(std::fabs(raw_left), std::fabs(raw_right));
  if (!std::isfinite(peak) || peak <= 0.0) {
    return CommandDecision{false, "command_out_of_range", stop};
  }
  const auto wheel_speed_limited = peak > config_.max_rear_wheel_speed_mps;
  const auto wheel_speed_scale = wheel_speed_limited ?
    config_.max_rear_wheel_speed_mps / peak : 1.0;
  const auto applied_speed = linear_x * wheel_speed_scale;
  if (!std::isfinite(wheel_speed_scale) || wheel_speed_scale <= 0.0 ||
    wheel_speed_scale > 1.0 || !std::isfinite(applied_speed) ||
    !std::isfinite(raw_left * wheel_speed_scale) ||
    !std::isfinite(raw_right * wheel_speed_scale))
  {
    return CommandDecision{false, "command_out_of_range", stop};
  }

  ControlCommand command;
  command.sequence = next_command_sequence_++;
  command.mode = ControlMode::Velocity;
  command.equivalent_steering_rad = applied_steering;
  command.rear_left_velocity_mps = raw_left * wheel_speed_scale;
  command.rear_right_velocity_mps = raw_right * wheel_speed_scale;
  current_command_ = command;
  has_last_command_ = true;
  last_command_sec_ = now_sec;
  status_.state = AdapterState::Active;
  status_.safe_stop_active = false;
  CommandDecision decision{true, "", command};
  decision.requested_speed_mps = linear_x;
  decision.applied_speed_mps = applied_speed;
  decision.requested_steering_rad = requested_steering;
  decision.applied_steering_rad = applied_steering;
  decision.wheel_speed_scale = wheel_speed_scale;
  decision.steering_limited = applied_steering != requested_steering;
  decision.wheel_speed_limited = wheel_speed_limited;
  return decision;
}

ControlCommand AdapterCore::tick(double now_sec)
{
  if (!backend_open_) {
    current_command_ = make_stop_command(true);
    return current_command_;
  }

  if (!config_.allow_motion || status_.state == AdapterState::Fault ||
    status_.state == AdapterState::Degraded || !status_.feedback_valid || safety_latched_)
  {
    has_last_command_ = false;
    current_command_ = make_stop_command(true);
    if (!config_.allow_motion) {
      status_.safe_stop_active = true;
    }
    return current_command_;
  }

  if (
    has_last_feedback_ && now_sec - last_feedback_sec_ > config_.feedback_timeout_sec)
  {
    status_.state = AdapterState::Fault;
    status_.safe_stop_active = true;
    status_.feedback_valid = false;
    has_last_command_ = false;
    current_command_ = make_stop_command(true);
    return current_command_;
  }

  if (has_heartbeat_ && now_sec - last_heartbeat_change_sec_ > config_.heartbeat_timeout_sec) {
    status_.state = AdapterState::Fault;
    status_.safe_stop_active = true;
    status_.heartbeat_ok = false;
    status_.fault_latched = true;
    safety_latched_ = true;
    has_last_command_ = false;
    current_command_ = make_stop_command(true);
    return current_command_;
  }

  if (status_.state == AdapterState::Fault || status_.state == AdapterState::Degraded) {
    current_command_ = make_stop_command(true);
    return current_command_;
  }

  // Hold the explicit sequence-zero initialization stop until the adapter has
  // observed fresh feedback.  This makes the epoch barrier observable to late
  // subscribers without allocating sequence one as an implicit startup stop.
  if (status_.state == AdapterState::Connecting && !has_last_feedback_) {
    return current_command_;
  }

  if (has_last_command_ && now_sec - last_command_sec_ > config_.command_timeout_sec) {
    has_last_command_ = false;
    status_.state = status_.feedback_valid ? AdapterState::Ready : AdapterState::Connecting;
    status_.safe_stop_active = true;
    current_command_ = make_stop_command(true);
    return current_command_;
  }

  if (status_.state == AdapterState::Ready && !has_last_command_) {
    if (keepalive_pending_ && !keepalive_acknowledged_) {
      return current_command_;
    }
    // Sequence zero is only the initialization stop.  Once fresh feedback
    // has made the adapter Ready, emit a new explicit zero command so a
    // lockstep plant can distinguish the next applied sample from a stale
    // initialization keepalive.
    current_command_ = make_stop_command(true);
    keepalive_pending_ = true;
    keepalive_acknowledged_ = false;
    has_last_command_ = true;
    last_command_sec_ = now_sec;
    status_.state = AdapterState::Active;
    return current_command_;
  }
  if (status_.state != AdapterState::Active) {
    current_command_ = make_stop_command(true);
  }
  return current_command_;
}

const AdapterStatus & AdapterCore::status() const
{
  return status_;
}

const OdometryState & AdapterCore::odometry() const
{
  return odometry_;
}

ControlCommand AdapterCore::make_stop_command(bool safe_stop_active)
{
  ControlCommand command;
  command.sequence = next_command_sequence_++;
  command.mode = ControlMode::Stop;
  command.safe_stop_active = safe_stop_active;
  return command;
}

void AdapterCore::rebase_odometry(const FeedbackObservation & observation)
{
  ++status_.odometry_rebase_count;
  odometry_.valid = false;
  odometry_.linear_mps = 0.0;
  odometry_.angular_radps = 0.0;
  if (
    observation.health_present &&
    observation.left_position_valid &&
    observation.right_position_valid &&
    std::isfinite(observation.left_position_mrad) &&
    std::isfinite(observation.right_position_mrad))
  {
    last_left_position_mrad_ = observation.left_position_mrad;
    last_right_position_mrad_ = observation.right_position_mrad;
    last_device_time_ms_ = observation.device_time_ms;
    odometry_baseline_valid_ = true;
  } else {
    last_device_time_ms_.reset();
    odometry_baseline_valid_ = false;
  }
  status_.odometry_baseline_valid = odometry_baseline_valid_;
}

void AdapterCore::update_odometry(const FeedbackObservation & observation)
{
  if (
    !observation.health_present ||
    !observation.left_position_valid ||
    !observation.right_position_valid ||
    !std::isfinite(observation.left_position_mrad) ||
    !std::isfinite(observation.right_position_mrad) ||
    !config_valid())
  {
    odometry_.valid = false;
    odometry_baseline_valid_ = false;
    status_.odometry_baseline_valid = false;
    return;
  }

  if (!odometry_baseline_valid_ || !last_device_time_ms_.has_value()) {
    last_left_position_mrad_ = observation.left_position_mrad;
    last_right_position_mrad_ = observation.right_position_mrad;
    last_device_time_ms_ = observation.device_time_ms;
    odometry_baseline_valid_ = true;
    status_.odometry_baseline_valid = true;
    odometry_.linear_mps = 0.0;
    odometry_.angular_radps = 0.0;
    odometry_.valid = true;
    return;
  }

  const auto device_delta_ms = static_cast<std::uint32_t>(
    observation.device_time_ms - *last_device_time_ms_);
  if (device_delta_ms == 0U || device_delta_ms > 0x7FFFFFFFU) {
    rebase_odometry(observation);
    return;
  }

  const auto dt = static_cast<double>(device_delta_ms) / 1000.0;
  if (!std::isfinite(dt) || dt <= 0.0 || dt > config_.max_odometry_step_sec) {
    rebase_odometry(observation);
    return;
  }

  const bool position_reset =
    std::fabs(observation.left_position_mrad) <= 1e-9 &&
    std::fabs(observation.right_position_mrad) <= 1e-9 &&
    (std::fabs(last_left_position_mrad_) > 1e-9 ||
    std::fabs(last_right_position_mrad_) > 1e-9);
  if (position_reset) {
    rebase_odometry(observation);
    return;
  }

  const auto d_left =
    static_cast<double>(config_.left_position_sign) *
    (observation.left_position_mrad - last_left_position_mrad_) / 1000.0 *
    config_.wheel_radius_m;
  const auto d_right =
    static_cast<double>(config_.right_position_sign) *
    (observation.right_position_mrad - last_right_position_mrad_) / 1000.0 *
    config_.wheel_radius_m;
  if (
    !std::isfinite(d_left) ||
    !std::isfinite(d_right) ||
    std::fabs(d_left) > config_.max_position_delta_m ||
    std::fabs(d_right) > config_.max_position_delta_m)
  {
    rebase_odometry(observation);
    return;
  }

  const auto d_center = (d_left + d_right) / 2.0;
  const auto d_yaw = (d_right - d_left) / config_.rear_track_m;
  const auto mid_yaw = odometry_.yaw_rad + d_yaw / 2.0;
  const auto next_x = odometry_.x_m + d_center * std::cos(mid_yaw);
  const auto next_y = odometry_.y_m + d_center * std::sin(mid_yaw);
  auto next_yaw = odometry_.yaw_rad + d_yaw;
  while (next_yaw >= M_PI) {
    next_yaw -= 2.0 * M_PI;
  }
  while (next_yaw < -M_PI) {
    next_yaw += 2.0 * M_PI;
  }

  const auto linear_mps = d_center / dt;
  const auto angular_radps = d_yaw / dt;
  if (
    !std::isfinite(next_x) ||
    !std::isfinite(next_y) ||
    !std::isfinite(next_yaw) ||
    !std::isfinite(linear_mps) ||
    !std::isfinite(angular_radps))
  {
    rebase_odometry(observation);
    return;
  }

  odometry_.x_m = next_x;
  odometry_.y_m = next_y;
  odometry_.yaw_rad = next_yaw;
  odometry_.linear_mps = linear_mps;
  odometry_.angular_radps = angular_radps;
  odometry_.valid = true;
  last_left_position_mrad_ = observation.left_position_mrad;
  last_right_position_mrad_ = observation.right_position_mrad;
  last_device_time_ms_ = observation.device_time_ms;
  odometry_baseline_valid_ = true;
  status_.odometry_baseline_valid = true;
}

bool AdapterCore::config_valid() const
{
  return std::isfinite(config_.wheelbase_m) && config_.wheelbase_m > 0.0 &&
         std::isfinite(config_.rear_track_m) && config_.rear_track_m > 0.0 &&
         std::isfinite(config_.max_equivalent_steering_angle_rad) &&
         config_.max_equivalent_steering_angle_rad > 0.0 &&
         config_.max_equivalent_steering_angle_rad < kHalfPi &&
         std::isfinite(config_.max_rear_wheel_speed_mps) &&
         config_.max_rear_wheel_speed_mps > 0.0 &&
         std::isfinite(config_.command_timeout_sec) && config_.command_timeout_sec > 0.0 &&
         std::isfinite(config_.feedback_timeout_sec) && config_.feedback_timeout_sec > 0.0 &&
         std::isfinite(config_.heartbeat_timeout_sec) && config_.heartbeat_timeout_sec > 0.0 &&
         std::isfinite(config_.max_odometry_step_sec) && config_.max_odometry_step_sec > 0.0 &&
         std::isfinite(config_.wheel_radius_m) && config_.wheel_radius_m > 0.0 &&
         (config_.left_position_sign == 1 || config_.left_position_sign == -1) &&
         (config_.right_position_sign == 1 || config_.right_position_sign == -1) &&
         std::isfinite(config_.max_position_delta_m) && config_.max_position_delta_m > 0.0;
}

}  // namespace car_hardware_adapter
