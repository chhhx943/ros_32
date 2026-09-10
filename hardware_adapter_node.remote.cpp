#include "car_hardware_adapter/adapter_core.hpp"
#include "car_hardware_adapter/mock_backend.hpp"
#include "car_hardware_adapter/protocol_codec.hpp"
#ifdef __linux__
#include "car_hardware_adapter/socketcan_backend.hpp"
#endif

#include <chrono>
#include <cmath>
#include <limits>
#include <map>
#include <memory>
#include <optional>
#include <utility>
#include <stdexcept>
#include <string>
#include <vector>

#include "car_interfaces/msg/focus_state.hpp"
#include "car_interfaces/msg/hardware_feedback.hpp"
#include "car_interfaces/msg/hardware_status.hpp"
#include "car_interfaces/msg/runtime_epoch_state.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rcl_interfaces/msg/set_parameters_result.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_srvs/srv/trigger.hpp"

using namespace std::chrono_literals;

namespace
{

double duration_to_sec(const builtin_interfaces::msg::Time & stamp, double now_sec)
{
  const auto stamp_sec =
    static_cast<double>(stamp.sec) + static_cast<double>(stamp.nanosec) / 1000000000.0;
  return stamp_sec == 0.0 ? 0.0 : now_sec - stamp_sec;
}

std::string state_name(car_hardware_adapter::AdapterState state)
{
  switch (state) {
    case car_hardware_adapter::AdapterState::Init:
      return "INIT";
    case car_hardware_adapter::AdapterState::Connecting:
      return "CONNECTING";
    case car_hardware_adapter::AdapterState::Ready:
      return "READY";
    case car_hardware_adapter::AdapterState::Active:
      return "ACTIVE";
    case car_hardware_adapter::AdapterState::Degraded:
      return "DEGRADED";
    case car_hardware_adapter::AdapterState::Fault:
      return "FAULT";
  }
  return "UNKNOWN";
}

car_hardware_adapter::MockFault parse_mock_fault(const std::string & value)
{
  static const std::map<std::string, car_hardware_adapter::MockFault> faults{
    {"none", car_hardware_adapter::MockFault::None},
    {"stop_feedback", car_hardware_adapter::MockFault::StopFeedback},
    {"stop_heartbeat", car_hardware_adapter::MockFault::StopHeartbeat},
    {"left_encoder", car_hardware_adapter::MockFault::LeftEncoder},
    {"transport", car_hardware_adapter::MockFault::Transport}};
  const auto found = faults.find(value);
  if (found == faults.end()) {
    throw std::invalid_argument("unknown mock_fault: " + value);
  }
  return found->second;
}

}  // namespace

class HardwareAdapterNode : public rclcpp::Node
{
public:
  HardwareAdapterNode()
  : Node("hardware_adapter"),
    core_(car_hardware_adapter::AdapterConfig{
    declare_parameter("wheelbase_m", 0.1417),
    declare_parameter("rear_track_m", 0.120),
    declare_parameter("max_equivalent_steering_angle_rad", 0.3490658504),
    declare_parameter("max_rear_wheel_speed_mps", 0.600),
    declare_parameter("command_timeout_sec", 0.10),
    declare_parameter("feedback_timeout_sec", 0.10),
    declare_parameter("heartbeat_timeout_sec", 0.20),
    declare_parameter("max_odometry_step_sec", 0.10),
    declare_parameter("allow_motion", false),
    declare_parameter("wheel_radius_m", 0.03325),
    declare_parameter<int>("left_position_sign", 1),
    declare_parameter<int>("right_position_sign", 1),
    declare_parameter("max_position_delta_m", 0.20)})
  {
    backend_name_ = declare_parameter("backend", "mock");
    can_interface_ = declare_parameter("can_interface", "can0");
    lockstep_mode_ = declare_parameter("lockstep_mode", false);
    publish_rate_hz_ = declare_parameter("publish_rate_hz", 20.0);
    feedback_publish_rate_hz_ = declare_parameter("feedback_publish_rate_hz", 20.0);
    wheel_odom_frame_ = declare_parameter("wheel_odom_frame", "odom");
    wheel_odom_child_frame_ = declare_parameter("wheel_odom_child_frame", "base_link");
    drive_wheel_names_ = declare_parameter(
      "drive_wheel_names", std::vector<std::string>{"rear_left_drive", "rear_right_drive"});
    steering_actuator_name_ = declare_parameter("steering_actuator_name", "front_steering_servo");
    enable_mock_fault_injection_ = declare_parameter("enable_mock_fault_injection", false);
    mock_fault_name_ = declare_parameter("mock_fault", "none");
    suppress_wheel_odom_publish_ = declare_parameter("suppress_wheel_odom_publish", false);
    run_id_ = declare_parameter<std::string>("run_id", "");
    // A non-empty run_id is the D1 launch contract.  Gate the mock backend on
    // the epoch even if the parameter service has not yet reflected
    // use_sim_time during node construction.
    runtime_epoch_gate_enabled_ = !run_id_.empty();
    RCLCPP_INFO(
      get_logger(), "mock runtime epoch gate: enabled=%d run_id=%s",
      runtime_epoch_gate_enabled_, run_id_.c_str());
    if (backend_name_ != "mock" && enable_mock_fault_injection_) {
      throw std::invalid_argument("mock fault injection requires backend=mock");
    }
    if (!enable_mock_fault_injection_ && mock_fault_name_ != "none") {
      throw std::invalid_argument("mock fault injection is disabled");
    }

    if (backend_name_ == "mock") {
      backend_ = std::make_unique<car_hardware_adapter::MockBackend>(
        car_hardware_adapter::MockConfig{1.0 / feedback_publish_rate_hz_, 0.10});
    } else if (backend_name_ == "socketcan") {
#ifdef __linux__
      backend_ = std::make_unique<car_hardware_adapter::SocketCanBackend>(
        car_hardware_adapter::SocketCanConfig{can_interface_});
#else
      throw std::invalid_argument("backend=socketcan requires Linux SocketCAN");
#endif
    } else {
      throw std::invalid_argument("unsupported backend: " + backend_name_);
    }
    const auto opened = backend_->open();
    if (opened.ok) {
      core_.on_backend_open(transport_now_sec());
    } else {
      core_.on_backend_closed(opened.reason, transport_now_sec());
    }
    core_.on_backend_health(backend_->health(), transport_now_sec());
    if (enable_mock_fault_injection_) {
      auto * mock_backend = dynamic_cast<car_hardware_adapter::MockBackend *>(backend_.get());
      if (mock_backend == nullptr) {
        throw std::invalid_argument("mock fault injection requires a mock backend");
      }
      mock_backend->inject(parse_mock_fault(mock_fault_name_));
    }

    cmd_sub_ = create_subscription<geometry_msgs::msg::Twist>(
      "cmd_vel_limited",
      10,
      [this](const geometry_msgs::msg::Twist::SharedPtr msg) {
        const auto zero_twist = std::fabs(msg->linear.x) <= 1e-9 &&
        std::fabs(msg->angular.z) <= 1e-9;
        if (zero_twist && last_command_.mode == car_hardware_adapter::ControlMode::Stop &&
        last_feedback_.has_value() &&
        last_feedback_->applied_command_sequence == last_command_.sequence)
        {
          // Motion publishes zero continuously while parked. Once the stop
          // is applied, repeated zeros are idempotent and must not allocate
          // an unbounded stream of new lockstep sequences.
          return;
        }
        if (lockstep_mode_ && command_feedback_pending()) {
          // Do not overwrite a command which has been sent but not yet
          // applied.  Keeping only the newest desired twist is safe for the
          // controller stream, while the applied command sequence remains
          // strictly contiguous for the scenario plant.
          deferred_twist_ = std::make_pair(msg->linear.x, msg->angular.z);
          return;
        }
        accept_twist(msg->linear.x, msg->angular.z);
      });

    focus_sub_ = create_subscription<car_interfaces::msg::FocusState>(
      "focus/state",
      10,
      [this](const car_interfaces::msg::FocusState::SharedPtr msg) {
        focus_estop_active_ = msg->estop_active;
        focus_safe_stop_active_ = msg->safe_stop_active;
      });
    runtime_epoch_sub_ = create_subscription<car_interfaces::msg::RuntimeEpochState>(
      "/car/runtime/epoch_state", rclcpp::QoS(1).reliable().transient_local(),
      [this](const car_interfaces::msg::RuntimeEpochState::SharedPtr message) {
        if (!runtime_epoch_gate_enabled_ ||
        message->state != car_interfaces::msg::RuntimeEpochState::STATE_READY ||
        message->time_epoch == 0U || message->run_id != run_id_ ||
        message->time_epoch == runtime_epoch_seen_)
        {
          return;
        }
        runtime_epoch_seen_ = message->time_epoch;
        runtime_epoch_ready_ = true;
        core_.on_backend_open(transport_now_sec());
        // A new runtime epoch is a hard ownership boundary.  Do not carry a
        // deferred controller twist, command stamp, or applied-feedback
        // identity across it; the first command in the new epoch must be the
        // explicit sequence-zero stop.
        last_command_ = car_hardware_adapter::ControlCommand{};
        last_command_.safe_stop_active = true;
        last_feedback_.reset();
        deferred_twist_.reset();
        initial_stop_consumer_ready_ = !lockstep_mode_;
        last_command_stamp_ = builtin_interfaces::msg::Time{};
        last_feedback_stamp_ = builtin_interfaces::msg::Time{};
        RCLCPP_INFO(
          get_logger(), "accepted runtime epoch %lu; reset initial command sequence",
          static_cast<unsigned long>(runtime_epoch_seen_));
      });

    status_pub_ = create_publisher<car_interfaces::msg::HardwareStatus>("hardware/status", 10);
    feedback_pub_ =
      create_publisher<car_interfaces::msg::HardwareFeedback>("hardware/feedback", 10);
    odom_pub_ = create_publisher<nav_msgs::msg::Odometry>(
      "wheel_odom", rclcpp::QoS(1).reliable().durability_volatile());
    reconnect_service_ = create_service<std_srvs::srv::Trigger>(
      "/car/hardware/mock/reconnect",
      [this](
        const std::shared_ptr<std_srvs::srv::Trigger::Request>,
        std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
        // Backend reconnect and liveness bookkeeping use steady transport
        // time; ROS time is reserved for message stamps.
        // Mixing the two leaves the mock feedback poller in a future epoch.
        const auto result = backend_->reconnect(transport_now_sec());
        if (!result.ok) {
          core_.on_backend_health(backend_->health(), transport_now_sec());
          response->success = false;
          response->message = result.reason;
          return;
        }
        core_.on_backend_open(transport_now_sec());
        core_.on_backend_health(backend_->health(), transport_now_sec());
        // A reconnect starts a new transport health window.  Do not let a
        // prior connection's failed frames keep command_path_ok false after
        // fresh feedback and a new command group have been accepted.
        dropped_count_ = 0U;
        if (backend_name_ == "mock") {
          mock_fault_name_ = "none";
        }
        response->success = true;
        response->message = backend_name_ + " backend reconnected in safe stop";
      });
    parameter_callback_handle_ = add_on_set_parameters_callback(
      [this](const std::vector<rclcpp::Parameter> & parameters) {
        return on_parameters_set(parameters);
      });

    const auto period = std::chrono::duration<double>(1.0 / publish_rate_hz_);
    timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::milliseconds>(period),
      [this]() {update();});
  }

private:
  double now_sec() const
  {
    return now().seconds();
  }

  static double transport_now_sec()
  {
    return std::chrono::duration<double>(
      std::chrono::steady_clock::now().time_since_epoch()).count();
  }

  void update()
  {
    const auto stamp = now();
    // The mock backend must not manufacture zero-stamped feedback while a
    // simulated clock is still starting.  A real backend must continue its
    // own safety/transport loop even if the ROS clock is unavailable.
    if (backend_name_ == "mock" &&
      !car_hardware_adapter::is_ros_time_ready(stamp.nanoseconds()))
    {
      return;
    }
    if (runtime_epoch_gate_enabled_ && !runtime_epoch_ready_) {
      return;
    }
    // The simulation clock is intentionally held at a step while scenario
    // waits for its observation barrier.  Mock transport still needs to
    // complete its CAN exchange and emit the applied feedback for that step;
    // use receipt time for transport deadlines while retaining ROS time for
    // every published stamp and simulation safety decision.
    const auto transport_seconds = transport_now_sec();

    // The scenario holds /clock at the current step until the matching sensor
    // observation arrives.  Once a command has been applied, do not let the
    // adapter allocate a second keepalive sequence on that same simulated
    // stamp: that would look like a new plant step without a new observation.
    // The next wall-timer iteration will call core_.tick again after the
    // scenario publishes the next clock stamp.
    const auto same_ros_stamp =
      rclcpp::Time(last_command_stamp_).nanoseconds() == stamp.nanoseconds();
    const auto feedback_pending = this->command_feedback_pending();
    const auto hold_applied_command =
      lockstep_mode_ && same_ros_stamp && last_command_.sequence != 0U &&
      last_feedback_.has_value() &&
      last_feedback_->applied_command_sequence == last_command_.sequence;
    // In the deterministic mock transport, never let the host allocate a
    // second command before the applied feedback for the current command has
    // become observable.  Otherwise two complete CAN groups can be applied
    // between feedback polls and the scenario would see sequence N+2 without
    // ever observing N+1.
    const auto control_seconds = transport_seconds;
    // Consume health from the previous transport cycle before generating the
    // next command.  A CAN error frame is fail-closed and remains latched in
    // AdapterCore until an explicit backend reconnect.
    core_.on_backend_health(backend_->health(), control_seconds);
    const auto hold_initial_stop = lockstep_mode_ && !initial_stop_consumer_ready_;
    // A controller callback can arrive while the previous applied feedback is
    // still being delivered to the scenario.  Keep that desired twist
    // deferred until the scenario publishes the next clock stamp; otherwise
    // accepting it here allocates a new command sequence on the old plant
    // step and its feedback is stale by the time the scenario consumes it.
    if (lockstep_mode_ && deferred_twist_.has_value() &&
      !hold_initial_stop && !feedback_pending && !same_ros_stamp)
    {
      const auto deferred = *deferred_twist_;
      deferred_twist_.reset();
      accept_twist(deferred.first, deferred.second);
    }
    const auto command_feedback_waiting = this->command_feedback_pending();
    auto command = (hold_initial_stop || command_feedback_waiting || hold_applied_command) ?
      last_command_ : core_.tick(control_seconds);
    if (focus_estop_active_) {
      command.mode = car_hardware_adapter::ControlMode::Stop;
      command.estop_active = true;
      command.safe_stop_active = false;
      command.equivalent_steering_rad = 0.0;
      command.rear_left_velocity_mps = 0.0;
      command.rear_right_velocity_mps = 0.0;
    } else if (focus_safe_stop_active_) {
      command.mode = car_hardware_adapter::ControlMode::Stop;
      command.safe_stop_active = true;
      command.equivalent_steering_rad = 0.0;
      command.rear_left_velocity_mps = 0.0;
      command.rear_right_velocity_mps = 0.0;
    }

    const auto encoded = codec_.encode_control_group(command);
    if (encoded.ok) {
      const auto result = backend_->send_control_group(encoded.frames, transport_seconds);
      if (result.ok) {
        tx_count_ += result.frames_sent;
        const auto command_is_retransmission =
          lockstep_mode_ && command.sequence == last_command_.sequence &&
          last_command_stamp_.sec != 0;
        last_command_ = command;
        if (!command_is_retransmission) {
          // In lockstep mode this is the generation stamp of the command, not
          // the wall-time stamp of a transport keepalive retransmission.
          last_command_stamp_ = stamp;
        }
      } else {
        ++dropped_count_;
      }
    } else {
      ++dropped_count_;
    }

    std::optional<car_hardware_adapter::FeedbackObservation> polled_observation;
    const auto polled_frames = backend_->poll(transport_seconds);
    for (const auto & frame : polled_frames) {
      ++rx_count_;
      const auto result = codec_.ingest_feedback_frame(frame);
      if (result.status != car_hardware_adapter::DecodeStatus::Ok) {
        ++dropped_count_;
        continue;
      }
      if (result.observation.has_value()) {
        polled_observation = *result.observation;
      }
    }
    core_.on_backend_health(backend_->health(), control_seconds);
    const auto new_feedback_observation = polled_observation.has_value() &&
      (!last_feedback_.has_value() ||
      polled_observation->applied_command_sequence != last_feedback_->applied_command_sequence);
    if (polled_observation.has_value()) {
      // Feed every transport observation to the core so heartbeat and
      // feedback liveness remain current.  Only a new applied command
      // sequence is a plant observation and is published to lockstep
      // consumers/odometry.
      last_feedback_stamp_ = stamp;
      core_.on_feedback(*polled_observation, control_seconds);
      last_feedback_ = *polled_observation;
      const auto relay_initial_stop = lockstep_mode_ && !initial_stop_consumer_ready_ &&
        polled_observation->applied_command_sequence == 0U;
      if (!lockstep_mode_ || new_feedback_observation || relay_initial_stop) {
        // The feedback stamp is the transport/application observation time.
        // A command may be generated while the previous scenario step is
        // held and applied on the next step; publishing its generation stamp
        // would make a valid applied sample look stale to lockstep.
        publish_feedback(*polled_observation, stamp);
        if (relay_initial_stop && feedback_pub_->get_subscription_count() > 0U) {
          initial_stop_consumer_ready_ = true;
        }
      }
    }
    publish_status(stamp);
    // In lockstep mode an unchanged applied feedback is a retransmission, not
    // a new simulation observation.  Re-publishing odometry with the same ROS
    // stamp makes C6 reject an otherwise valid stream as non-monotonic and can
    // prevent AMCL from observing the next frame.  Publish odometry exactly
    // once for each backend observation; normal runtime keeps its periodic
    // odometry publication semantics.
    if (!lockstep_mode_ || new_feedback_observation || last_command_.sequence == 0U) {
      publish_odometry(stamp);
    }
  }

  void publish_feedback(
    const car_hardware_adapter::FeedbackObservation & observation,
    const rclcpp::Time & stamp)
  {
    car_interfaces::msg::HardwareFeedback msg;
    msg.header.stamp = stamp;
    msg.header.frame_id = "base_link";
    msg.sequence = observation.sequence;
    msg.applied_command_sequence = observation.applied_command_sequence;
    msg.drive_wheel_names = drive_wheel_names_;
    msg.drive_wheel_velocity_mps = {
      static_cast<float>(observation.left_velocity_mps),
      static_cast<float>(observation.right_velocity_mps)};
    const auto nan = std::numeric_limits<float>::quiet_NaN();
    msg.drive_wheel_position_rad = {
      observation.left_position_valid ? static_cast<float>(observation.left_position_mrad /
      1000.0) : nan,
      observation.right_position_valid ? static_cast<float>(observation.right_position_mrad /
      1000.0) : nan};
    msg.drive_wheel_current_a = {nan, nan};
    msg.steering_actuator_name = steering_actuator_name_;
    msg.steering_command_rad = static_cast<float>(last_command_.equivalent_steering_rad);
    msg.steering_position_rad = nan;
    msg.steering_position_valid = false;
    msg.valid = observation.velocity_present && core_.status().state !=
      car_hardware_adapter::AdapterState::Fault;
    msg.fault_code = observation.fault_code;
    msg.fault_reason = car_hardware_adapter::describe_fault(observation.fault_code).fault_reason;
    feedback_pub_->publish(msg);
  }

  bool command_feedback_pending() const
  {
    return lockstep_mode_ && last_command_.sequence != 0U &&
           (!last_feedback_.has_value() ||
           last_feedback_->applied_command_sequence != last_command_.sequence);
  }

  void accept_twist(double linear_x, double angular_z)
  {
    if (lockstep_mode_ && !initial_stop_consumer_ready_) {
      deferred_twist_ = std::make_pair(linear_x, angular_z);
      return;
    }
    const auto decision = core_.on_twist(
      linear_x, angular_z, transport_now_sec());
    if (decision.accepted) {
      const auto command_is_retransmission =
        lockstep_mode_ && decision.command.sequence == last_command_.sequence &&
        last_command_stamp_.sec != 0;
      last_command_ = decision.command;
      if (!command_is_retransmission) {
        last_command_stamp_ = now();
      }
    }
  }

  void publish_status(const rclcpp::Time & stamp)
  {
    const auto & status = core_.status();
    const auto backend_health = backend_->health();
    const auto fault = car_hardware_adapter::describe_fault(status.fault_code);
    const auto now_seconds = now_sec();

    car_interfaces::msg::HardwareStatus msg;
    msg.header.stamp = stamp;
    msg.header.frame_id = "base_link";
    msg.backend = backend_name_;
    msg.transport = backend_name_ == "socketcan" ? "socketcan_raw" : "mock_chassis";
    msg.state = state_name(status.state);
    msg.connected = backend_health.connected;
    msg.command_path_ok = backend_health.connected && backend_health.writable &&
      backend_health.error_class == car_hardware_adapter::TransportErrorClass::None &&
      dropped_count_ == 0U;
    msg.feedback_valid = status.feedback_valid;
    msg.rear_encoder_feedback_ok = !last_feedback_.has_value() ||
      ((last_feedback_->status_flags & 0x06U) == 0x06U);
    msg.steering_command_accepted =
      last_feedback_.has_value() && (last_feedback_->status_flags & 0x08U) != 0U;
    msg.steering_feedback_available = false;
    msg.heartbeat_ok = status.heartbeat_ok && backend_health.heartbeat_ok;
    msg.estop_active = focus_estop_active_ ||
      (last_feedback_.has_value() && (last_feedback_->status_flags & 0x20U) != 0U);
    msg.safe_stop_active =
      focus_safe_stop_active_ || status.safe_stop_active || backend_health.safe_stop_active;
    msg.fault_latched = status.fault_latched;
    msg.fault_code = status.fault_code;
    msg.fault_reason = fault.fault_reason;
    msg.recovery_hint = fault.recovery_hint;
    msg.command_age_sec =
      static_cast<float>(duration_to_sec(last_command_stamp_, now_seconds));
    msg.feedback_age_sec =
      static_cast<float>(duration_to_sec(last_feedback_stamp_, now_seconds));
    msg.tx_count = tx_count_;
    msg.rx_count = rx_count_;
    msg.dropped_count = dropped_count_;
    msg.last_command_sequence = last_command_.sequence;
    msg.last_feedback_sequence = last_feedback_.has_value() ? last_feedback_->sequence : 0U;
    msg.feedback_sequence_gap_count = status.feedback_sequence_gap_count;
    msg.odometry_rebase_count = status.odometry_rebase_count;
    msg.odometry_baseline_valid = status.odometry_baseline_valid;
    status_pub_->publish(msg);
  }

  void publish_odometry(const rclcpp::Time & stamp)
  {
    if (suppress_wheel_odom_publish_) {
      return;
    }
    const auto & odometry = core_.odometry();
    if (!odometry.valid) {
      return;
    }
    nav_msgs::msg::Odometry msg;
    msg.header.stamp = stamp;
    msg.header.frame_id = wheel_odom_frame_;
    msg.child_frame_id = wheel_odom_child_frame_;
    msg.pose.pose.position.x = odometry.x_m;
    msg.pose.pose.position.y = odometry.y_m;
    msg.pose.pose.orientation.z = std::sin(odometry.yaw_rad / 2.0);
    msg.pose.pose.orientation.w = std::cos(odometry.yaw_rad / 2.0);
    msg.twist.twist.linear.x = odometry.linear_mps;
    msg.twist.twist.angular.z = odometry.angular_radps;
    // Keep the real robot_localization input well-defined. Zero covariance
    // is not a deterministic confidence value for an enabled EKF variable.
    msg.pose.covariance[0] = 0.01;
    msg.pose.covariance[7] = 0.01;
    msg.pose.covariance[35] = 0.01;
    msg.twist.covariance[0] = 0.01;
    msg.twist.covariance[35] = 0.01;
    odom_pub_->publish(msg);
  }

  rcl_interfaces::msg::SetParametersResult on_parameters_set(
    const std::vector<rclcpp::Parameter> & parameters)
  {
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;
    auto * mock = dynamic_cast<car_hardware_adapter::MockBackend *>(backend_.get());
    std::optional<std::string> requested_fault;
    std::optional<bool> requested_suppression;
    try {
      for (const auto & parameter : parameters) {
        if (parameter.get_name() == "mock_fault") {
          requested_fault = parameter.as_string();
          parse_mock_fault(*requested_fault);
        } else if (parameter.get_name() == "suppress_wheel_odom_publish") {
          requested_suppression = parameter.as_bool();
        }
      }
      if (requested_fault.has_value() && (!enable_mock_fault_injection_ || mock == nullptr)) {
        throw std::invalid_argument("mock fault injection is disabled");
      }
      if (requested_fault.has_value()) {
        mock->inject(parse_mock_fault(*requested_fault));
        mock_fault_name_ = *requested_fault;
      }
      if (requested_suppression.has_value()) {
        suppress_wheel_odom_publish_ = *requested_suppression;
      }
    } catch (const std::exception & exception) {
      result.successful = false;
      result.reason = exception.what();
    }
    return result;
  }

  std::string backend_name_{"mock"};
  std::string can_interface_{"can0"};
  bool lockstep_mode_{false};
  double publish_rate_hz_{20.0};
  double feedback_publish_rate_hz_{20.0};
  std::string wheel_odom_frame_{"odom"};
  std::string wheel_odom_child_frame_{"base_link"};
  std::vector<std::string> drive_wheel_names_{"rear_left_drive", "rear_right_drive"};
  std::string steering_actuator_name_{"front_steering_servo"};
  bool focus_estop_active_{false};
  bool focus_safe_stop_active_{false};
  bool enable_mock_fault_injection_{false};
  bool suppress_wheel_odom_publish_{false};
  std::string mock_fault_name_{"none"};
  std::string run_id_;
  bool runtime_epoch_gate_enabled_{false};
  bool runtime_epoch_ready_{false};
  std::uint64_t runtime_epoch_seen_{0U};
  // Lockstep consumers must observe the explicit sequence-zero initialization
  // stop before the adapter can advance the command sequence.  This prevents
  // a late scenario subscription from seeing sequence one first and then
  // being unable to close its initialization barrier.
  bool initial_stop_consumer_ready_{false};
  std::uint64_t tx_count_{0};
  std::uint64_t rx_count_{0};
  std::uint64_t dropped_count_{0};
  builtin_interfaces::msg::Time last_command_stamp_;
  builtin_interfaces::msg::Time last_feedback_stamp_;
  car_hardware_adapter::ControlCommand last_command_;
  std::optional<car_hardware_adapter::FeedbackObservation> last_feedback_;
  std::optional<std::pair<double, double>> deferred_twist_;
  car_hardware_adapter::ProtocolCodec codec_;
  car_hardware_adapter::AdapterCore core_;
  std::unique_ptr<car_hardware_adapter::IChassisBackend> backend_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_sub_;
  rclcpp::Subscription<car_interfaces::msg::FocusState>::SharedPtr focus_sub_;
  rclcpp::Subscription<car_interfaces::msg::RuntimeEpochState>::SharedPtr runtime_epoch_sub_;
  rclcpp::Publisher<car_interfaces::msg::HardwareStatus>::SharedPtr status_pub_;
  rclcpp::Publisher<car_interfaces::msg::HardwareFeedback>::SharedPtr feedback_pub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr reconnect_service_;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr parameter_callback_handle_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<HardwareAdapterNode>());
  rclcpp::shutdown();
  return 0;
}
