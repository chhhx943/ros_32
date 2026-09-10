#pragma once

#include "car_hardware_adapter/protocol_codec.hpp"

#include <array>
#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

namespace car_hardware_adapter
{

enum class TransportErrorClass
{
  None,
  Warning,
  ErrorPassive,
  BusOff,
  TxFailure
};

struct BackendResult
{
  bool ok{false};
  std::string reason;
  std::size_t frames_sent{0U};
  double group_elapsed_sec{0.0};
};

struct BackendHealth
{
  bool connected{false};
  bool writable{false};
  bool safe_stop_active{true};
  bool heartbeat_ok{false};
  std::uint16_t fault_code{0};
  TransportErrorClass error_class{TransportErrorClass::None};
  std::size_t tx_count{0U};
  std::size_t rx_count{0U};
  std::size_t error_count{0U};
  std::size_t restart_count{0U};
  std::string last_error;
};

class IChassisBackend
{
public:
  virtual ~IChassisBackend() = default;

  virtual BackendResult open() = 0;
  virtual void close() = 0;
  virtual BackendResult send_control_group(
    const std::array<CanFrame, 2> & frames, double now_sec) = 0;
  // Legacy single-frame path retained for E-stop bypass and existing callers.
  virtual BackendResult send(const CanFrame & frame, double now_sec) = 0;
  virtual std::vector<CanFrame> poll(double now_sec) = 0;
  virtual BackendHealth health() const = 0;
  virtual BackendResult reconnect(double now_sec) = 0;
  virtual bool feedback_due() const {return false;}
};

}  // namespace car_hardware_adapter
