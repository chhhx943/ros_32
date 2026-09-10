#pragma once

#ifdef __linux__

#include "car_hardware_adapter/chassis_backend.hpp"

#include <linux/can.h>
#include <linux/can/error.h>

#include <array>
#include <cstdint>
#include <string>
#include <vector>

namespace car_hardware_adapter
{

struct SocketCanConfig
{
  std::string interface_name{"can0"};
};

struct SocketCanErrorInfo
{
  TransportErrorClass error_class{TransportErrorClass::None};
  std::uint16_t fault_code{0U};
  std::string reason;
  bool disconnect{false};
};

class SocketCanBackend final : public IChassisBackend
{
public:
  explicit SocketCanBackend(SocketCanConfig config = {});
  ~SocketCanBackend() override;

  BackendResult open() override;
  void close() override;
  BackendResult send_control_group(
    const std::array<CanFrame, 2> & frames, double now_sec) override;
  BackendResult send(const CanFrame & frame, double now_sec) override;
  std::vector<CanFrame> poll(double now_sec) override;
  BackendHealth health() const override;
  BackendResult reconnect(double now_sec) override;

  static bool to_native_frame(
    const CanFrame & source, struct can_frame & destination, std::string & reason);
  static CanFrame from_native_frame(const struct can_frame & source);
  static SocketCanErrorInfo classify_error_frame(const struct can_frame & frame);

  // Kept public so error-frame classification can be tested without a CAN device.
  void process_error_frame(const struct can_frame & frame);

private:
  SocketCanConfig config_;
  int socket_fd_{-1};
  BackendHealth health_{};

  void close_socket();
  void enter_fail_safe(
    std::uint16_t fault_code, TransportErrorClass error_class,
    const std::string & reason, bool disconnect);
  BackendResult fail_result(
    std::uint16_t fault_code, TransportErrorClass error_class,
    const std::string & reason, std::size_t frames_sent = 0U);
  BackendResult send_native_frame(const struct can_frame & frame);
  static bool valid_time(double now_sec);
  static std::string errno_reason(const std::string & prefix, int error_number);
};

}  // namespace car_hardware_adapter

#endif  // __linux__
