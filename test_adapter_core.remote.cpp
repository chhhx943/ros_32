#include "car_hardware_adapter/adapter_core.hpp"

#include <cmath>

#include <gtest/gtest.h>

using namespace car_hardware_adapter;

namespace
{

TEST(AdapterCoreTest, RosTimeReadinessRejectsZeroAndAcceptsPositiveTime)
{
  EXPECT_FALSE(is_ros_time_ready(0));
  EXPECT_FALSE(is_ros_time_ready(-1));
  EXPECT_TRUE(is_ros_time_ready(1));
}

AdapterConfig valid_adapter_config()
{
  auto config = AdapterConfig{0.1417, 0.120, 0.3490658504, 0.600, 0.10, 0.10, 0.20, 0.10};
  config.allow_motion = true;
  return config;
}

FeedbackObservation valid_velocity_observation(std::uint16_t sequence)
{
  FeedbackObservation observation;
  observation.sequence = sequence;
  observation.velocity_present = true;
  observation.status_present = true;
  observation.health_present = true;
  observation.heartbeat_counter = static_cast<std::uint8_t>(sequence & 0xFFU);
  observation.applied_command_sequence = sequence;
  observation.device_time_ms = static_cast<std::uint32_t>(sequence) * 20U;
  observation.left_velocity_mps = 0.2;
  observation.right_velocity_mps = 0.2;
  observation.left_velocity_valid = true;
  observation.right_velocity_valid = true;
  observation.left_position_valid = true;
  observation.right_position_valid = true;
  observation.left_position_mrad = static_cast<double>(sequence) * 1000.0;
  observation.right_position_mrad = static_cast<double>(sequence) * 1000.0;
  observation.status_flags = 0x0FU;
  observation.fault_code = 0x0000U;
  return observation;
}

FeedbackObservation valid_drive_diagnostics(std::uint16_t sequence)
{
  auto observation = valid_velocity_observation(sequence);
  observation.diagnostics_present = true;
  observation.diagnostics_values_known = true;
  observation.drive_allowed = true;
  observation.command_fresh = true;
  observation.diagnostic_flags = 0x06U;
  observation.safety_state = 4U;
  observation.safety_action = 1U;
  observation.can_error_class = 0U;
  return observation;
}

void make_ready(AdapterCore & core);

TEST(AdapterCoreTest, DefaultConfigurationDisablesMotion)
{
  AdapterCore core(AdapterConfig{0.30, 0.24, 0.45, 0.60, 0.10, 0.10, 0.20, 0.10});
  core.on_backend_open(0.0);
  core.on_feedback(valid_velocity_observation(1), 0.01);
  const auto result = core.on_twist(0.2, 0.0, 0.02);
  EXPECT_FALSE(result.accepted);
  EXPECT_EQ(result.reason, "motion_disabled");
  EXPECT_EQ(result.command.mode, ControlMode::Stop);
  EXPECT_TRUE(result.command.safe_stop_active);
  EXPECT_TRUE(core.status().safe_stop_active);
}

TEST(AdapterCoreTest, DisabledMotionFeedbackKeepsSafeStopActive)
{
  AdapterCore core(AdapterConfig{0.30, 0.24, 0.45, 0.60, 0.10, 0.10, 0.20, 0.10});
  core.on_backend_open(0.0);
  core.on_feedback(valid_velocity_observation(1), 0.01);
  EXPECT_TRUE(core.status().safe_stop_active);
  EXPECT_EQ(core.tick(0.02).mode, ControlMode::Stop);
  EXPECT_TRUE(core.status().safe_stop_active);
}

TEST(AdapterCoreTest, BackendHealthFaultsAndHealthyRecoveryNeedsFeedback)
{
  AdapterCore core(valid_adapter_config());
  core.on_backend_open(0.0);
  BackendHealth health;
  health.connected = false;
  core.on_backend_health(health, 0.01);
  EXPECT_EQ(core.status().state, AdapterState::Fault);
  EXPECT_EQ(core.tick(0.02).mode, ControlMode::Stop);

  health.connected = true;
  health.writable = true;
  health.heartbeat_ok = true;
  core.on_backend_health(health, 0.03);
  EXPECT_EQ(core.status().state, AdapterState::Fault);
  EXPECT_EQ(core.tick(0.04).mode, ControlMode::Stop);
  core.on_backend_open(0.045);
  core.on_feedback(valid_velocity_observation(1), 0.05);
  EXPECT_EQ(core.status().state, AdapterState::Ready);
}

TEST(AdapterCoreTest, DiagnosticsMustExplicitlyAllowDrive)
{
  AdapterCore core(valid_adapter_config());
  core.on_backend_open(0.0);
  auto observation = valid_velocity_observation(1);
  observation.diagnostics_present = true;
  observation.diagnostics_values_known = false;
  core.on_feedback(observation, 0.01);
  EXPECT_EQ(core.status().state, AdapterState::Fault);
  EXPECT_TRUE(core.tick(0.02).safe_stop_active);
}

TEST(AdapterCoreTest, FaultStaysLatchedAcrossHealthyBackendUntilReopen)
{
  AdapterCore core(valid_adapter_config());
  core.on_backend_open(0.0);
  BackendHealth bad;
  bad.connected = false;
  core.on_backend_health(bad, 0.01);
  BackendHealth healthy{true, true, false, true, 0, TransportErrorClass::None};
  core.on_backend_health(healthy, 0.02);
  EXPECT_EQ(core.status().state, AdapterState::Fault);
  EXPECT_EQ(core.tick(0.03).mode, ControlMode::Stop);
  core.on_backend_open(0.04);
  core.on_feedback(valid_velocity_observation(1), 0.05);
  EXPECT_EQ(core.status().state, AdapterState::Ready);
}

TEST(AdapterCoreTest, DegradedRecoveryRequiresFreshCompleteFeedback)
{
  AdapterCore core(valid_adapter_config());
  core.on_backend_open(0.0);
  BackendHealth degraded{true, true, true, true, 0, TransportErrorClass::None};
  core.on_backend_health(degraded, 0.01);
  EXPECT_EQ(core.status().state, AdapterState::Degraded);
  BackendHealth healthy{true, true, false, true, 0, TransportErrorClass::None};
  core.on_backend_health(healthy, 0.02);
  EXPECT_EQ(core.status().state, AdapterState::Connecting);
  EXPECT_EQ(core.tick(0.03).mode, ControlMode::Stop);
  core.on_feedback(valid_drive_diagnostics(1), 0.04);
  EXPECT_EQ(core.status().state, AdapterState::Ready);
}

TEST(AdapterCoreTest, BackendSafeStopHealthIsDegraded)
{
  AdapterCore core(valid_adapter_config());
  core.on_backend_open(0.0);
  BackendHealth health{true, true, true, true, 0, TransportErrorClass::None};
  core.on_backend_health(health, 0.01);
  EXPECT_EQ(core.status().state, AdapterState::Degraded);
  EXPECT_TRUE(core.status().safe_stop_active);
  EXPECT_EQ(core.tick(0.02).mode, ControlMode::Stop);
}

TEST(AdapterCoreTest, BackendHeartbeatFailureLatchesFaultUntilReopen)
{
  AdapterCore core(valid_adapter_config());
  core.on_backend_open(0.0);
  BackendHealth health{true, true, false, false, 0, TransportErrorClass::None};
  core.on_backend_health(health, 0.01);
  EXPECT_EQ(core.status().state, AdapterState::Fault);
  EXPECT_TRUE(core.status().fault_latched);
  EXPECT_TRUE(core.status().safe_stop_active);
  EXPECT_EQ(core.tick(0.02).mode, ControlMode::Stop);
  health.heartbeat_ok = true;
  core.on_backend_health(health, 0.03);
  EXPECT_EQ(core.status().state, AdapterState::Fault);
  EXPECT_EQ(core.tick(0.04).mode, ControlMode::Stop);
  core.on_backend_open(0.05);
  core.on_feedback(valid_velocity_observation(1), 0.06);
  EXPECT_EQ(core.status().state, AdapterState::Ready);
}

TEST(AdapterCoreTest, BackendCloseLatchesFaultUntilReopen)
{
  AdapterCore core(valid_adapter_config());
  make_ready(core);
  ASSERT_TRUE(core.on_twist(0.2, 0.0, 0.02).accepted);
  core.on_backend_closed("closed", 0.03);
  EXPECT_EQ(core.status().state, AdapterState::Fault);
  EXPECT_TRUE(core.status().fault_latched);
  EXPECT_EQ(core.tick(0.04).mode, ControlMode::Stop);
  EXPECT_EQ(core.on_twist(0.2, 0.0, 0.05).reason, "adapter_not_ready");
  core.on_backend_open(0.06);
  core.on_feedback(valid_velocity_observation(1), 0.07);
  EXPECT_EQ(core.status().state, AdapterState::Ready);
}

TEST(AdapterCoreTest, SafeStopHealthDoesNotDowngradeLatchedFault)
{
  AdapterCore core(valid_adapter_config());
  core.on_backend_open(0.0);
  BackendHealth bad;
  bad.connected = false;
  core.on_backend_health(bad, 0.01);
  BackendHealth safe{true, true, true, true, 0, TransportErrorClass::None};
  core.on_backend_health(safe, 0.02);
  EXPECT_EQ(core.status().state, AdapterState::Fault);
  EXPECT_TRUE(core.status().fault_latched);
  EXPECT_EQ(core.tick(0.03).mode, ControlMode::Stop);
}

TEST(AdapterCoreTest, NormalFeedbackCannotClearFaultLatch)
{
  AdapterCore core(valid_adapter_config());
  core.on_backend_open(0.0);
  BackendHealth bad;
  bad.connected = false;
  core.on_backend_health(bad, 0.01);
  core.on_feedback(valid_velocity_observation(1), 0.02);
  EXPECT_EQ(core.status().state, AdapterState::Fault);
  EXPECT_TRUE(core.status().fault_latched);
  EXPECT_TRUE(core.tick(0.03).safe_stop_active);
}

TEST(AdapterCoreTest, FeedbackSafeStopBlocksMotionUntilClear)
{
  AdapterCore core(valid_adapter_config());
  core.on_backend_open(0.0);
  auto stopped = valid_velocity_observation(1);
  stopped.status_flags = 0x4FU;
  core.on_feedback(stopped, 0.01);

  EXPECT_EQ(core.status().state, AdapterState::Degraded);
  EXPECT_TRUE(core.status().safe_stop_active);
  EXPECT_EQ(core.on_twist(0.2, 0.0, 0.02).reason, "adapter_not_ready");
  EXPECT_EQ(core.tick(0.03).mode, ControlMode::Stop);

  core.on_feedback(valid_velocity_observation(2), 0.04);
  EXPECT_EQ(core.status().state, AdapterState::Ready);
  EXPECT_TRUE(core.on_twist(0.2, 0.0, 0.05).accepted);
}

TEST(AdapterCoreTest, DiagnosticsSafetyMatrixGatesMotion)
{
  const auto check_degraded = [](FeedbackObservation observation) {
      AdapterCore core(valid_adapter_config());
      core.on_backend_open(0.0);
      core.on_feedback(observation, 0.01);
      EXPECT_EQ(core.status().state, AdapterState::Degraded);
      EXPECT_TRUE(core.status().safe_stop_active);
      EXPECT_TRUE(core.odometry().valid);
      EXPECT_EQ(core.tick(0.02).mode, ControlMode::Stop);
      EXPECT_FALSE(core.on_twist(0.2, 0.0, 0.02).accepted);
    };
  const auto check_fault = [](FeedbackObservation observation) {
      AdapterCore core(valid_adapter_config());
      core.on_backend_open(0.0);
      core.on_feedback(observation, 0.01);
      EXPECT_EQ(core.status().state, AdapterState::Fault);
      EXPECT_TRUE(core.status().safe_stop_active);
      EXPECT_FALSE(core.odometry().valid);
      EXPECT_EQ(core.tick(0.02).mode, ControlMode::Stop);
    };

  auto calibration = valid_drive_diagnostics(1);
  calibration.calibration_required = true;
  check_degraded(calibration);
  auto disallowed = valid_drive_diagnostics(1);
  disallowed.drive_allowed = false;
  check_degraded(disallowed);
  auto stale = valid_drive_diagnostics(1);
  stale.command_fresh = false;
  check_degraded(stale);
  auto can_error = valid_drive_diagnostics(1);
  can_error.can_error_class = 2U;
  check_fault(can_error);
  auto high_risk = valid_drive_diagnostics(1);
  high_risk.diagnostic_flags = 0x46U;
  check_fault(high_risk);
  auto wrong_state = valid_drive_diagnostics(1);
  wrong_state.safety_state = 2U;
  check_degraded(wrong_state);
  auto wrong_action = valid_drive_diagnostics(1);
  wrong_action.safety_action = 2U;
  check_degraded(wrong_action);

  AdapterCore normal(valid_adapter_config());
  normal.on_backend_open(0.0);
  normal.on_feedback(valid_drive_diagnostics(1), 0.01);
  EXPECT_EQ(normal.status().state, AdapterState::Ready);
  ASSERT_TRUE(normal.on_twist(0.2, 0.0, 0.02).accepted);
  EXPECT_EQ(normal.tick(0.03).mode, ControlMode::Velocity);

  AdapterCore legacy(valid_adapter_config());
  legacy.on_backend_open(0.0);
  legacy.on_feedback(valid_velocity_observation(1), 0.01);
  EXPECT_EQ(legacy.status().state, AdapterState::Ready);
}

TEST(AdapterCoreTest, DisabledMotionChecksFiniteBeforeReadiness)
{
  AdapterCore core(AdapterConfig{0.30, 0.24, 0.45, 0.60, 0.10, 0.10, 0.20, 0.10});
  const auto result = core.on_twist(NAN, 0.0, 0.0);
  EXPECT_EQ(result.reason, "command_not_finite");
  const auto disabled = core.on_twist(0.1, 0.0, 0.0);
  EXPECT_EQ(disabled.reason, "adapter_not_ready");
  EXPECT_EQ(disabled.command.mode, ControlMode::Stop);
  EXPECT_EQ(core.status().state, AdapterState::Fault);
  EXPECT_TRUE(disabled.command.safe_stop_active);
}

TEST(AdapterCoreTest, MissingPositionInvalidatesOdometryAndFaults)
{
  AdapterCore core(valid_adapter_config());
  core.on_backend_open(0.0);
  auto first = valid_velocity_observation(1);
  core.on_feedback(first, 0.01);
  ASSERT_TRUE(core.odometry().valid);
  auto missing = valid_velocity_observation(2);
  missing.right_position_valid = false;
  missing.applied_command_sequence = 99;
  core.on_feedback(missing, 0.02);
  EXPECT_EQ(core.status().state, AdapterState::Fault);
  EXPECT_TRUE(core.status().safe_stop_active);
  EXPECT_FALSE(core.odometry().valid);
  EXPECT_NEAR(core.odometry().x_m, 0.0, 1e-9);
}

TEST(AdapterCoreTest, InvalidVelocityFlagsForceFaultAndInvalidateOdometry)
{
  AdapterCore core(valid_adapter_config());
  core.on_backend_open(0.0);
  auto observation = valid_velocity_observation(1);
  observation.left_velocity_valid = false;
  core.on_feedback(observation, 0.01);
  EXPECT_EQ(core.status().state, AdapterState::Fault);
  EXPECT_TRUE(core.status().safe_stop_active);
  EXPECT_EQ(core.tick(0.02).mode, ControlMode::Stop);
  EXPECT_FALSE(core.odometry().valid);
}

TEST(AdapterCoreTest, NonFiniteVelocityForcesFaultBeforeReady)
{
  AdapterCore core(valid_adapter_config());
  core.on_backend_open(0.0);
  auto observation = valid_velocity_observation(1);
  observation.right_velocity_mps = INFINITY;
  core.on_feedback(observation, 0.01);
  EXPECT_EQ(core.status().state, AdapterState::Fault);
  EXPECT_TRUE(core.status().safe_stop_active);
  EXPECT_FALSE(core.odometry().valid);
}

TEST(AdapterCoreTest, NonFiniteCommandClearsActiveVelocity)
{
  AdapterCore core(valid_adapter_config());
  make_ready(core);
  ASSERT_TRUE(core.on_twist(0.2, 0.0, 0.02).accepted);
  EXPECT_EQ(core.tick(0.03).mode, ControlMode::Velocity);
  EXPECT_EQ(core.on_twist(NAN, 0.0, 0.04).reason, "command_not_finite");
  EXPECT_EQ(core.tick(0.05).mode, ControlMode::Stop);
  EXPECT_TRUE(core.tick(0.06).safe_stop_active);
  EXPECT_EQ(core.on_twist(0.0, INFINITY, 0.07).reason, "command_not_finite");
  EXPECT_EQ(core.tick(0.08).mode, ControlMode::Stop);
}

TEST(AdapterCoreTest, NonFiniteCommandLatchesSafetyUntilReopen)
{
  AdapterCore core(valid_adapter_config());
  make_ready(core);
  ASSERT_TRUE(core.on_twist(0.2, 0.0, 0.02).accepted);
  EXPECT_EQ(core.on_twist(INFINITY, 0.0, 0.03).reason, "command_not_finite");
  EXPECT_EQ(core.status().state, AdapterState::Fault);
  EXPECT_TRUE(core.status().safe_stop_active);
  EXPECT_EQ(core.on_twist(0.2, 0.0, 0.04).reason, "adapter_not_ready");
  EXPECT_EQ(core.tick(0.05).mode, ControlMode::Stop);
  EXPECT_EQ(core.tick(0.06).mode, ControlMode::Stop);
  core.on_backend_open(0.07);
  core.on_feedback(valid_velocity_observation(1), 0.08);
  EXPECT_TRUE(core.on_twist(0.2, 0.0, 0.09).accepted);
}

TEST(AdapterCoreTest, NonFiniteCommandTimeLatchesSafetyUntilReopen)
{
  AdapterCore core(valid_adapter_config());
  make_ready(core);

  const auto result = core.on_twist(0.2, 0.0, NAN);
  EXPECT_FALSE(result.accepted);
  EXPECT_EQ(result.reason, "command_time_not_finite");
  EXPECT_EQ(core.status().state, AdapterState::Fault);
  EXPECT_TRUE(core.status().fault_latched);
  EXPECT_EQ(core.tick(0.03).mode, ControlMode::Stop);
}

TEST(AdapterCoreTest, DeviceTimeResetRebasesWithoutAJump)
{
  AdapterCore core(valid_adapter_config());
  core.on_backend_open(0.0);
  auto first = valid_velocity_observation(1);
  first.device_time_ms = 1000U;
  first.left_position_mrad = 0.0;
  first.right_position_mrad = 0.0;
  core.on_feedback(first, 0.01);
  auto reset = valid_velocity_observation(2);
  reset.device_time_ms = 0U;
  reset.left_position_mrad = 1000.0;
  reset.right_position_mrad = 1000.0;
  core.on_feedback(reset, 0.02);
  EXPECT_FALSE(core.odometry().valid);
  EXPECT_NEAR(core.odometry().x_m, 0.0, 1e-9);
  EXPECT_TRUE(core.status().odometry_baseline_valid);
  auto next = valid_velocity_observation(3);
  next.device_time_ms = 20U;
  next.left_position_mrad = 2000.0;
  next.right_position_mrad = 2000.0;
  core.on_feedback(next, 0.03);
  EXPECT_TRUE(core.odometry().valid);
  EXPECT_NEAR(core.odometry().x_m, 0.03325, 1e-9);
}

TEST(AdapterCoreTest, FeedbackSequenceReinitializationRebases)
{
  AdapterCore core(valid_adapter_config());
  core.on_backend_open(0.0);
  auto first = valid_velocity_observation(100);
  first.device_time_ms = 1000U;
  first.left_position_mrad = 1000.0;
  first.right_position_mrad = 1000.0;
  core.on_feedback(first, 0.01);
  auto reset = valid_velocity_observation(0);
  reset.device_time_ms = 1020U;
  reset.left_position_mrad = 2000.0;
  reset.right_position_mrad = 2000.0;
  core.on_feedback(reset, 0.02);
  EXPECT_FALSE(core.odometry().valid);
  EXPECT_NEAR(core.odometry().x_m, 0.0, 1e-9);
  EXPECT_TRUE(core.status().odometry_baseline_valid);
}

TEST(AdapterCoreTest, InvalidDeviceIntervalRebasesAndUsesNextSample)
{
  AdapterCore core(valid_adapter_config());
  core.on_backend_open(0.0);
  auto first = valid_velocity_observation(1);
  first.device_time_ms = 1000U;
  first.left_position_mrad = 0.0;
  first.right_position_mrad = 0.0;
  core.on_feedback(first, 0.01);
  auto invalid = valid_velocity_observation(2);
  invalid.device_time_ms = 1000U;
  invalid.left_position_mrad = 1000.0;
  invalid.right_position_mrad = 1000.0;
  core.on_feedback(invalid, 0.02);
  EXPECT_FALSE(core.odometry().valid);
  auto next = valid_velocity_observation(3);
  next.device_time_ms = 1020U;
  next.left_position_mrad = 2000.0;
  next.right_position_mrad = 2000.0;
  core.on_feedback(next, 0.03);
  EXPECT_TRUE(core.odometry().valid);
  EXPECT_NEAR(core.odometry().x_m, 0.03325, 1e-9);
}

void make_ready(AdapterCore & core)
{
  core.on_backend_open(0.0);
  core.on_feedback(valid_velocity_observation(1), 0.01);
}

TEST(AdapterCoreTest, WheelRadiusAndEncoderMathReferenceValues)
{
  constexpr double radius_m = 0.03325;
  EXPECT_NEAR(2.0 * M_PI / 56000.0 * radius_m, 3.7306413e-6, 1e-10);
  EXPECT_NEAR((1000.0 / 1000.0) * radius_m, radius_m, 1e-12);
  EXPECT_NEAR(2.0 * M_PI * radius_m, 0.208915, 1e-6);
}

TEST(AdapterCoreTest, PositionSignsNormalizeForwardTravel)
{
  auto config = valid_adapter_config();
  config.right_position_sign = -1;
  AdapterCore core(config);
  core.on_backend_open(0.0);
  auto first = valid_velocity_observation(1);
  first.device_time_ms = 1000U;
  first.left_position_mrad = 0.0;
  first.right_position_mrad = 0.0;
  core.on_feedback(first, 0.01);
  auto forward = valid_velocity_observation(2);
  forward.device_time_ms = 1020U;
  forward.left_position_mrad = 1000.0;
  forward.right_position_mrad = -1000.0;
  core.on_feedback(forward, 0.02);
  EXPECT_NEAR(core.odometry().x_m, 0.03325, 1e-9);
  EXPECT_NEAR(core.odometry().yaw_rad, 0.0, 1e-9);
}

TEST(AdapterCoreTest, ForwardSequenceGapUsesCumulativePositionAndRecordsDiagnostic)
{
  AdapterCore core(valid_adapter_config());
  core.on_backend_open(0.0);
  auto first = valid_velocity_observation(10);
  first.device_time_ms = 1000U;
  first.left_position_mrad = 0.0;
  first.right_position_mrad = 0.0;
  core.on_feedback(first, 0.01);
  auto gap = valid_velocity_observation(12);
  gap.device_time_ms = 1040U;
  gap.left_position_mrad = 2000.0;
  gap.right_position_mrad = 2000.0;
  core.on_feedback(gap, 0.02);
  EXPECT_TRUE(core.odometry().valid);
  EXPECT_NEAR(core.odometry().x_m, 0.0665, 1e-9);
  EXPECT_EQ(core.status().feedback_sequence_gap_count, 1U);
}

TEST(AdapterCoreTest, PositionResetAndImpossibleJumpRebaseWithoutPoseJump)
{
  AdapterCore core(valid_adapter_config());
  core.on_backend_open(0.0);
  auto first = valid_velocity_observation(1);
  first.device_time_ms = 1000U;
  first.left_position_mrad = 1000.0;
  first.right_position_mrad = 1000.0;
  core.on_feedback(first, 0.01);
  auto reset = valid_velocity_observation(2);
  reset.device_time_ms = 1020U;
  reset.left_position_mrad = 0.0;
  reset.right_position_mrad = 0.0;
  core.on_feedback(reset, 0.02);
  EXPECT_FALSE(core.odometry().valid);
  EXPECT_NEAR(core.odometry().x_m, 0.0, 1e-9);
  auto jump = valid_velocity_observation(3);
  jump.device_time_ms = 1040U;
  jump.left_position_mrad = 10000.0;
  jump.right_position_mrad = 10000.0;
  core.on_feedback(jump, 0.03);
  EXPECT_FALSE(core.odometry().valid);
  EXPECT_NEAR(core.odometry().x_m, 0.0, 1e-9);
  EXPECT_GE(core.status().odometry_rebase_count, 2U);
}

TEST(AdapterCoreTest, IntegratesCircularArcWithMidpointHeading)
{
  AdapterCore core(valid_adapter_config());
  core.on_backend_open(0.0);
  auto first = valid_velocity_observation(1);
  first.device_time_ms = 1000U;
  first.left_position_mrad = 0.0;
  first.right_position_mrad = 0.0;
  core.on_feedback(first, 0.01);
  auto arc = valid_velocity_observation(2);
  arc.device_time_ms = 1020U;
  arc.left_position_mrad = 1000.0;
  arc.right_position_mrad = 2000.0;
  core.on_feedback(arc, 0.02);
  const auto d_yaw = (0.0665 - 0.03325) / 0.120;
  const auto d_center = (0.0665 + 0.03325) / 2.0;
  EXPECT_NEAR(core.odometry().x_m, d_center * std::cos(d_yaw / 2.0), 1e-9);
  EXPECT_NEAR(core.odometry().y_m, d_center * std::sin(d_yaw / 2.0), 1e-9);
  EXPECT_NEAR(core.odometry().yaw_rad, d_yaw, 1e-9);
}

TEST(AdapterCoreTest, MathematicalCounterRotatingWheelsIntegrateInPlace)
{
  AdapterCore core(valid_adapter_config());
  core.on_backend_open(0.0);
  auto first = valid_velocity_observation(1);
  first.device_time_ms = 1000U;
  first.left_position_mrad = 0.0;
  first.right_position_mrad = 0.0;
  core.on_feedback(first, 0.01);
  auto turn = valid_velocity_observation(2);
  turn.device_time_ms = 1020U;
  turn.left_position_mrad = 1000.0;
  turn.right_position_mrad = -1000.0;
  core.on_feedback(turn, 0.02);
  EXPECT_NEAR(core.odometry().x_m, 0.0, 1e-9);
  EXPECT_NEAR(core.odometry().y_m, 0.0, 1e-9);
  EXPECT_NEAR(core.odometry().yaw_rad, -0.5541666667, 1e-9);
}

}  // namespace

TEST(AdapterCoreTest, StartsFailClosedAndRequiresFreshFeedbackAndCommand)
{
  AdapterCore core(valid_adapter_config());
  EXPECT_EQ(core.status().state, AdapterState::Init);
  core.on_backend_open(0.0);
  EXPECT_EQ(core.status().state, AdapterState::Connecting);
  core.on_feedback(valid_velocity_observation(1), 0.01);
  EXPECT_EQ(core.status().state, AdapterState::Ready);
  core.on_twist(0.2, 0.1, 0.02);
  EXPECT_EQ(core.status().state, AdapterState::Active);
}

TEST(AdapterCoreTest, FirstAppliedCommandIsTheKnownZeroStopSequence)
{
  AdapterCore core(valid_adapter_config());
  core.on_backend_open(1.0);

  const auto command = core.tick(1.0);

  EXPECT_EQ(command.mode, ControlMode::Stop);
  EXPECT_TRUE(command.safe_stop_active);
  EXPECT_EQ(command.sequence, 0U);
}

TEST(AdapterCoreTest, RuntimeReopenStartsAFreshKnownZeroSequence)
{
  AdapterCore core(valid_adapter_config());
  core.on_backend_open(1.0);
  ASSERT_EQ(core.tick(1.0).sequence, 0U);
  core.on_feedback(valid_velocity_observation(1), 1.01);
  ASSERT_TRUE(core.on_twist(0.2, 0.0, 1.02).accepted);

  core.on_backend_open(2.0);

  EXPECT_EQ(core.tick(2.0).sequence, 0U);
}

TEST(AdapterCoreTest, ZeroKeepaliveDoesNotConsumeInitializationSequence)
{
  AdapterCore core(valid_adapter_config());
  make_ready(core);

  ASSERT_TRUE(core.on_twist(0.0, 0.0, 1.0).accepted);
  const auto first_motion = core.on_twist(0.2, 0.0, 1.01);

  EXPECT_TRUE(first_motion.accepted);
  EXPECT_EQ(first_motion.command.sequence, 1U);
}

TEST(AdapterCoreTest, FirstPostInitializationKeepaliveGetsANewSequence)
{
  AdapterCore core(valid_adapter_config());
  make_ready(core);

  const auto keepalive = core.tick(1.0);

  EXPECT_EQ(keepalive.mode, ControlMode::Stop);
  EXPECT_TRUE(keepalive.safe_stop_active);
  EXPECT_EQ(keepalive.sequence, 1U);
}

TEST(AdapterCoreTest, DoesNotIssueAnotherKeepaliveBeforeThePreviousOneIsApplied)
{
  AdapterCore core(valid_adapter_config());
  make_ready(core);

  const auto first = core.tick(0.02);
  ASSERT_EQ(first.sequence, 1U);
  ASSERT_TRUE(core.on_twist(0.0, 0.0, 0.021).accepted);

  EXPECT_EQ(core.tick(0.03).sequence, 1U);
}

TEST(AdapterCoreTest, RejectsZeroRadiusAndOutputsStop)
{
  AdapterCore core(valid_adapter_config());
  make_ready(core);
  const auto result = core.on_twist(0.0, 0.5, 1.0);
  EXPECT_FALSE(result.accepted);
  EXPECT_EQ(result.reason, "unsupported_zero_radius_command");
  EXPECT_EQ(result.command.mode, ControlMode::Stop);
  EXPECT_DOUBLE_EQ(result.wheel_speed_scale, 1.0);
  EXPECT_DOUBLE_EQ(result.applied_speed_mps, 0.0);
}

TEST(AdapterCoreTest, RejectsSteeringLimitAtOrAboveHalfPiAsInvalidConfiguration)
{
  auto config = valid_adapter_config();
  config.max_equivalent_steering_angle_rad = 1.5707963267948966;
  AdapterCore core(config);
  make_ready(core);

  const auto result = core.on_twist(0.20, 0.10, 1.0);

  EXPECT_FALSE(result.accepted);
  EXPECT_EQ(result.reason, "vehicle_geometry_unconfigured");
  EXPECT_EQ(result.command.mode, ControlMode::Stop);
  EXPECT_TRUE(result.command.safe_stop_active);
  EXPECT_NE(core.status().state, AdapterState::Active);
}

TEST(AdapterCoreTest, RejectsInvalidMaximumRearWheelSpeedConfiguration)
{
  const auto check_invalid_speed = [](double max_speed) {
      auto config = valid_adapter_config();
      config.max_rear_wheel_speed_mps = max_speed;
      AdapterCore core(config);
      make_ready(core);

      const auto result = core.on_twist(0.20, 0.10, 1.0);

      EXPECT_FALSE(result.accepted);
      EXPECT_EQ(result.reason, "vehicle_geometry_unconfigured");
      EXPECT_EQ(result.command.mode, ControlMode::Stop);
      EXPECT_TRUE(result.command.safe_stop_active);
      EXPECT_NE(core.status().state, AdapterState::Active);
    };

  check_invalid_speed(0.0);
  check_invalid_speed(-0.1);
  check_invalid_speed(NAN);
  check_invalid_speed(INFINITY);
}

TEST(AdapterCoreTest, AcceptsLowSpeedAckermannArcAndDerivesSteering)
{
  AdapterCore core(valid_adapter_config());
  make_ready(core);
  constexpr double speed = 0.12;
  constexpr double yaw_rate = 0.18;
  const auto result = core.on_twist(speed, yaw_rate, 0.02);
  ASSERT_TRUE(result.accepted);
  EXPECT_NEAR(
    (result.command.rear_left_velocity_mps + result.command.rear_right_velocity_mps) / 2.0,
    speed, 1e-12);
  EXPECT_NEAR(
    result.command.equivalent_steering_rad,
    std::atan(0.1417 * yaw_rate / speed), 1e-12);
  EXPECT_GT(result.command.equivalent_steering_rad, 0.0);
  EXPECT_EQ(core.tick(0.03).mode, ControlMode::Velocity);
}

TEST(AdapterCoreTest, R3xGeometryClampsSteeringAndAuditsRequestedAndAppliedValues)
{
  AdapterCore core(valid_adapter_config());
  make_ready(core);

  constexpr double requested_speed = 0.20;
  constexpr double requested_yaw_rate = 1.0;
  const auto result = core.on_twist(requested_speed, requested_yaw_rate, 1.0);

  ASSERT_TRUE(result.accepted);
  const auto requested_delta = std::atan(0.1417 * requested_yaw_rate / requested_speed);
  EXPECT_DOUBLE_EQ(result.requested_speed_mps, requested_speed);
  EXPECT_DOUBLE_EQ(result.requested_steering_rad, requested_delta);
  EXPECT_DOUBLE_EQ(result.applied_steering_rad, 0.3490658504);
  EXPECT_TRUE(result.steering_limited);
  EXPECT_FALSE(result.wheel_speed_limited);
  EXPECT_DOUBLE_EQ(result.wheel_speed_scale, 1.0);
  EXPECT_DOUBLE_EQ(result.applied_speed_mps, requested_speed);
  EXPECT_NEAR(result.command.equivalent_steering_rad, 0.3490658504, 1e-12);
}

TEST(AdapterCoreTest, R3xHighSpeedTurnUsesUniformWheelScalingAndIsAccepted)
{
  AdapterCore core(valid_adapter_config());
  make_ready(core);

  constexpr double requested_speed = 0.60;
  constexpr double requested_yaw_rate = 0.60;
  const auto result = core.on_twist(requested_speed, requested_yaw_rate, 1.0);

  ASSERT_TRUE(result.accepted);
  ASSERT_TRUE(result.wheel_speed_limited);
  EXPECT_FALSE(result.steering_limited);
  EXPECT_TRUE(std::isfinite(result.wheel_speed_scale));
  EXPECT_GT(result.wheel_speed_scale, 0.0);
  EXPECT_LE(result.wheel_speed_scale, 1.0);
  EXPECT_TRUE(std::isfinite(result.command.rear_left_velocity_mps));
  EXPECT_TRUE(std::isfinite(result.command.rear_right_velocity_mps));
  EXPECT_LT(result.wheel_speed_scale, 1.0);
  EXPECT_LE(
    std::max(
      std::fabs(result.command.rear_left_velocity_mps),
      std::fabs(result.command.rear_right_velocity_mps)), 0.600);
  EXPECT_NEAR(result.applied_speed_mps, requested_speed * result.wheel_speed_scale, 1e-12);

  const auto raw_left = requested_speed *
    (1.0 - std::tan(result.command.equivalent_steering_rad) * 0.120 / 2.0 / 0.1417);
  const auto raw_right = requested_speed *
    (1.0 + std::tan(result.command.equivalent_steering_rad) * 0.120 / 2.0 / 0.1417);
  EXPECT_NEAR(
    result.command.rear_left_velocity_mps / result.command.rear_right_velocity_mps,
    raw_left / raw_right, 1e-12);
}

TEST(AdapterCoreTest, R3xNegativeSpeedPreservesWheelSignsAndInnerOuterRelation)
{
  AdapterCore core(valid_adapter_config());
  make_ready(core);

  const auto result = core.on_twist(-0.60, 0.60, 1.0);

  ASSERT_TRUE(result.accepted);
  EXPECT_LT(result.command.rear_left_velocity_mps, 0.0);
  EXPECT_LT(result.command.rear_right_velocity_mps, 0.0);
  EXPECT_GT(
    std::fabs(result.command.rear_left_velocity_mps),
    std::fabs(result.command.rear_right_velocity_mps));
  EXPECT_NEAR(result.applied_speed_mps, -0.60 * result.wheel_speed_scale, 1e-12);
  EXPECT_LE(
    std::max(
      std::fabs(result.command.rear_left_velocity_mps),
      std::fabs(result.command.rear_right_velocity_mps)), 0.600);
}

TEST(AdapterCoreTest, CommandTimeoutOutputsSafeStop)
{
  AdapterCore core(valid_adapter_config());
  make_ready(core);
  ASSERT_TRUE(core.on_twist(0.2, 0.0, 0.02).accepted);
  core.on_feedback(valid_velocity_observation(2), 0.11);

  const auto command = core.tick(0.121);

  EXPECT_EQ(command.mode, ControlMode::Stop);
  EXPECT_TRUE(command.safe_stop_active);
  EXPECT_TRUE(core.status().safe_stop_active);
  EXPECT_EQ(core.status().state, AdapterState::Ready);
}

TEST(AdapterCoreTest, FeedbackTimeoutLatchesFaultAndOutputsStop)
{
  AdapterCore core(valid_adapter_config());
  make_ready(core);

  const auto command = core.tick(0.120);

  EXPECT_EQ(command.mode, ControlMode::Stop);
  EXPECT_TRUE(command.safe_stop_active);
  EXPECT_EQ(core.status().state, AdapterState::Fault);
}

TEST(AdapterCoreTest, HeartbeatStallLatchesFaultAndOutputsStop)
{
  AdapterCore core(valid_adapter_config());
  make_ready(core);
  auto stalled = valid_velocity_observation(2);
  stalled.heartbeat_counter = 1;
  core.on_feedback(stalled, 0.09);

  auto still_stalled = valid_velocity_observation(3);
  still_stalled.heartbeat_counter = 1;
  core.on_feedback(still_stalled, 0.22);

  EXPECT_EQ(core.status().state, AdapterState::Fault);
  EXPECT_TRUE(core.status().fault_latched);
  EXPECT_TRUE(core.status().safe_stop_active);
  EXPECT_EQ(core.tick(0.22).mode, ControlMode::Stop);
  EXPECT_EQ(core.on_twist(0.2, 0.0, 0.23).reason, "adapter_not_ready");
  EXPECT_EQ(core.tick(0.24).mode, ControlMode::Stop);
  core.on_backend_open(0.25);
  core.on_feedback(valid_velocity_observation(4), 0.26);
  EXPECT_EQ(core.status().state, AdapterState::Ready);
}

TEST(AdapterCoreTest, EncoderInvalidForcesFault)
{
  AdapterCore core(valid_adapter_config());
  core.on_backend_open(0.0);
  auto observation = valid_velocity_observation(1);
  observation.status_flags = 0x0DU;

  core.on_feedback(observation, 0.01);

  EXPECT_EQ(core.status().state, AdapterState::Fault);
  EXPECT_TRUE(core.status().safe_stop_active);
}

TEST(AdapterCoreTest, ReconnectRequiresFreshCommand)
{
  AdapterCore core(valid_adapter_config());
  make_ready(core);
  ASSERT_TRUE(core.on_twist(0.2, 0.0, 0.02).accepted);
  core.on_backend_closed("transport_closed", 0.03);
  core.on_backend_open(0.04);
  core.on_feedback(valid_velocity_observation(2), 0.05);

  EXPECT_EQ(core.status().state, AdapterState::Ready);
  EXPECT_EQ(core.tick(0.06).mode, ControlMode::Stop);
}

TEST(AdapterCoreTest, FaultFeedbackOutputsStop)
{
  AdapterCore core(valid_adapter_config());
  make_ready(core);
  auto fault = valid_velocity_observation(2);
  fault.fault_code = 0x0009U;
  fault.status_flags = 0x87U;
  core.on_feedback(fault, 0.03);

  const auto command = core.tick(0.03);

  EXPECT_EQ(command.mode, ControlMode::Stop);
  EXPECT_TRUE(command.safe_stop_active);
  EXPECT_EQ(core.status().state, AdapterState::Fault);
  EXPECT_EQ(core.status().fault_code, 0x0009U);
}

TEST(AdapterCoreTest, IntegratesPositionMradThroughWheelRadius)
{
  AdapterCore core(valid_adapter_config());
  core.on_backend_open(0.0);

  auto first = valid_velocity_observation(1);
  first.left_velocity_mps = 0.0;
  first.right_velocity_mps = 0.0;
  first.left_position_mrad = 0.0;
  first.right_position_mrad = 0.0;
  first.device_time_ms = 1000U;
  core.on_feedback(first, 0.01);

  auto second = first;
  second.sequence = 2U;
  second.heartbeat_counter = 2U;
  second.left_position_mrad = 1000.0;
  second.right_position_mrad = 1000.0;
  second.device_time_ms = 1020U;
  core.on_feedback(second, 0.03);

  ASSERT_TRUE(core.odometry().valid);
  EXPECT_NEAR(core.odometry().x_m, 0.03325, 1e-9);
  EXPECT_NEAR(core.odometry().y_m, 0.0, 1e-9);
  EXPECT_NEAR(core.odometry().yaw_rad, 0.0, 1e-9);
}

TEST(AdapterCoreTest, IntegratesRearWheelOdometryFromCumulativePosition)
{
  AdapterCore core(valid_adapter_config());
  make_ready(core);
  core.on_feedback(valid_velocity_observation(2), 0.06);

  const auto odom = core.odometry();

  ASSERT_TRUE(odom.valid);
  EXPECT_NEAR(odom.x_m, 0.03325, 1e-9);
  EXPECT_NEAR(odom.y_m, 0.0, 1e-9);
  EXPECT_NEAR(odom.yaw_rad, 0.0, 1e-9);
  EXPECT_NEAR(odom.linear_mps, 1.6625, 1e-9);
  EXPECT_DOUBLE_EQ(odom.angular_radps, 0.0);
}

TEST(AdapterCoreTest, DuplicateFeedbackRefreshesLivenessWithoutReintegratingOdometry)
{
  auto config = valid_adapter_config();
  config.max_odometry_step_sec = 0.50;
  AdapterCore core(config);
  make_ready(core);
  core.on_feedback(valid_velocity_observation(1), 0.15);

  const auto odom = core.odometry();

  EXPECT_TRUE(core.status().feedback_valid);
  EXPECT_NE(core.status().state, AdapterState::Fault);
  EXPECT_NEAR(odom.x_m, 0.0, 1e-9);
  EXPECT_NEAR(odom.y_m, 0.0, 1e-9);
  EXPECT_NEAR(odom.yaw_rad, 0.0, 1e-9);
}

TEST(AdapterCoreTest, OdomDtAlwaysUsesDeviceTime)
{
  AdapterCore core(valid_adapter_config());
  core.on_backend_open(0.0);
  core.on_feedback(valid_velocity_observation(1), 0.01);
  core.on_feedback(valid_velocity_observation(2), 0.06);

  EXPECT_TRUE(core.odometry().valid);
  EXPECT_NEAR(core.odometry().x_m, 0.03325, 1e-9);
  EXPECT_TRUE(std::isfinite(core.odometry().x_m));
  EXPECT_TRUE(std::isfinite(core.odometry().y_m));
  EXPECT_TRUE(std::isfinite(core.odometry().yaw_rad));
  EXPECT_TRUE(std::isfinite(core.odometry().linear_mps));
  EXPECT_TRUE(std::isfinite(core.odometry().angular_radps));
}

TEST(AdapterCoreTest, HostTimeDoesNotOverrideDeviceTime)
{
  AdapterCore core(valid_adapter_config());
  core.on_backend_open(0.0);
  core.on_feedback(valid_velocity_observation(1), 0.01, 10.0);
  core.on_feedback(valid_velocity_observation(2), 0.06, 10.05);
  EXPECT_TRUE(core.odometry().valid);
  EXPECT_NEAR(core.odometry().x_m, 0.03325, 1e-9);
}

TEST(AdapterCoreTest, DuplicateFeedbackSequenceDoesNotIntegrateAppliedSequenceChange)
{
  AdapterCore core(valid_adapter_config());
  core.on_backend_open(0.0);
  auto first = valid_velocity_observation(1);
  core.on_feedback(first, 0.01, 0.0);
  auto duplicate = first;
  duplicate.applied_command_sequence = 2;
  core.on_feedback(duplicate, 0.02, 0.02);
  EXPECT_NEAR(core.odometry().x_m, 0.0, 1e-9);
  auto next = duplicate;
  next.sequence = 2;
  next.heartbeat_counter = 2;
  next.device_time_ms = 40U;
  next.left_position_mrad = 2000.0;
  next.right_position_mrad = 2000.0;
  core.on_feedback(next, 0.03, 0.04);
  EXPECT_NEAR(core.odometry().x_m, 0.03325, 1e-9);
}
