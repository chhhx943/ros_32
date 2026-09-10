#ifdef __linux__

#include "car_hardware_adapter/socketcan_backend.hpp"

#include <chrono>
#include <cerrno>
#include <cmath>
#include <cstring>
#include <fcntl.h>
#include <linux/can/raw.h>
#include <net/if.h>
#include <poll.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <unistd.h>

#include <algorithm>
#include <utility>

namespace car_hardware_adapter
{

namespace
{

constexpr std::uint16_t kFaultCanWarning = 0x000BU;
constexpr std::uint16_t kFaultErrorPassive = 0x0008U;
constexpr std::uint16_t kFaultBusOff = 0x000AU;
constexpr std::uint16_t kFaultTxFailure = 0x0009U;

bool has_control_state(const struct can_frame & frame, std::uint8_t mask)
{
  return (frame.can_id & CAN_ERR_CRTL) != 0U && (frame.data[1] & mask) != 0U;
}

}  // namespace

SocketCanBackend::SocketCanBackend(SocketCanConfig config)
: config_(std::move(config))
{
  health_.safe_stop_active = true;
}

SocketCanBackend::~SocketCanBackend()
{
  close();
}

bool SocketCanBackend::valid_time(double now_sec)
{
  return std::isfinite(now_sec) && now_sec >= 0.0;
}

std::string SocketCanBackend::errno_reason(const std::string & prefix, int error_number)
{
  return prefix + ": " + std::strerror(error_number);
}

bool SocketCanBackend::to_native_frame(
  const CanFrame & source, struct can_frame & destination, std::string & reason)
{
  if (source.extended) {
    reason = "extended_id";
    return false;
  }
  if (source.rtr) {
    reason = "remote_frame";
    return false;
  }
  if (source.id > CAN_SFF_MASK) {
    reason = "invalid_id";
    return false;
  }
  if (source.data.size() != CAN_MAX_DLEN) {
    reason = "invalid_dlc";
    return false;
  }

  destination = {};
  destination.can_id = source.id;
  destination.can_dlc = CAN_MAX_DLEN;
  std::copy(source.data.begin(), source.data.end(), destination.data);
  reason.clear();
  return true;
}

CanFrame SocketCanBackend::from_native_frame(const struct can_frame & source)
{
  CanFrame result;
  result.id = source.can_id & CAN_SFF_MASK;
  result.extended = (source.can_id & CAN_EFF_FLAG) != 0U;
  result.rtr = (source.can_id & CAN_RTR_FLAG) != 0U;
  const auto dlc = std::min<std::uint8_t>(source.can_dlc, CAN_MAX_DLEN);
  result.data.assign(source.data, source.data + dlc);
  return result;
}

SocketCanErrorInfo SocketCanBackend::classify_error_frame(const struct can_frame & frame)
{
  const auto error_id = frame.can_id & CAN_ERR_MASK;
  if ((error_id & CAN_ERR_BUSOFF) != 0U) {
    return SocketCanErrorInfo{
      TransportErrorClass::BusOff, kFaultBusOff, "bus_off", true};
  }
  if (
    has_control_state(frame, CAN_ERR_CRTL_RX_PASSIVE) ||
    has_control_state(frame, CAN_ERR_CRTL_TX_PASSIVE))
  {
    return SocketCanErrorInfo{
      TransportErrorClass::ErrorPassive, kFaultErrorPassive, "error_passive", false};
  }
  if (
    (error_id & (CAN_ERR_ACK | CAN_ERR_TX_TIMEOUT | CAN_ERR_LOSTARB)) != 0U ||
    has_control_state(frame, CAN_ERR_CRTL_TX_OVERFLOW))
  {
    return SocketCanErrorInfo{
      TransportErrorClass::TxFailure, kFaultTxFailure, "tx_failure", false};
  }
  if (
    has_control_state(frame, CAN_ERR_CRTL_RX_WARNING) ||
    has_control_state(frame, CAN_ERR_CRTL_TX_WARNING))
  {
    return SocketCanErrorInfo{
      TransportErrorClass::Warning, kFaultCanWarning, "can_warning", false};
  }
  return SocketCanErrorInfo{
    TransportErrorClass::Warning, kFaultCanWarning, "can_error", false};
}

void SocketCanBackend::process_error_frame(const struct can_frame & frame)
{
  const auto info = classify_error_frame(frame);
  ++health_.error_count;
  health_.error_class = info.error_class;
  health_.fault_code = info.fault_code;
  health_.last_error = info.reason;
  health_.safe_stop_active = true;
  // Any kernel-reported CAN error is fail-closed.  Even a warning/passive
  // frame must not permit the ROS layer to continue writing until an explicit
  // reconnect starts a fresh health window.
  health_.writable = false;
  if (info.disconnect) {
    close_socket();
  }
}

void SocketCanBackend::close_socket()
{
  if (socket_fd_ >= 0) {
    ::close(socket_fd_);
    socket_fd_ = -1;
  }
  health_.connected = false;
  health_.writable = false;
  health_.heartbeat_ok = false;
}

void SocketCanBackend::close()
{
  close_socket();
  health_.safe_stop_active = true;
}

void SocketCanBackend::enter_fail_safe(
  std::uint16_t fault_code, TransportErrorClass error_class,
  const std::string & reason, bool disconnect)
{
  ++health_.error_count;
  health_.fault_code = fault_code;
  health_.error_class = error_class;
  health_.last_error = reason;
  health_.safe_stop_active = true;
  if (disconnect) {
    close_socket();
  } else {
    health_.writable = false;
  }
}

BackendResult SocketCanBackend::fail_result(
  std::uint16_t fault_code, TransportErrorClass error_class,
  const std::string & reason, std::size_t frames_sent)
{
  enter_fail_safe(fault_code, error_class, reason, true);
  return BackendResult{false, reason, frames_sent, 0.0};
}

BackendResult SocketCanBackend::open()
{
  close_socket();
  health_.safe_stop_active = true;
  health_.error_class = TransportErrorClass::None;
  health_.fault_code = 0U;
  health_.last_error.clear();

  if (config_.interface_name.empty() || config_.interface_name.size() >= IFNAMSIZ) {
    return fail_result(kFaultTxFailure, TransportErrorClass::TxFailure, "invalid_interface");
  }

  socket_fd_ = ::socket(PF_CAN, SOCK_RAW | SOCK_NONBLOCK, CAN_RAW);
  if (socket_fd_ < 0) {
    return fail_result(
      kFaultTxFailure, TransportErrorClass::TxFailure,
      errno_reason("socket", errno));
  }

  const can_err_mask_t error_mask = CAN_ERR_MASK;
  if (::setsockopt(
      socket_fd_, SOL_CAN_RAW, CAN_RAW_ERR_FILTER, &error_mask,
      sizeof(error_mask)) < 0)
  {
    const auto reason = errno_reason("setsockopt(CAN_RAW_ERR_FILTER)", errno);
    close_socket();
    return fail_result(kFaultTxFailure, TransportErrorClass::TxFailure, reason);
  }

  struct ifreq interface_request {};
  std::strncpy(
    interface_request.ifr_name, config_.interface_name.c_str(), IFNAMSIZ - 1U);
  if (::ioctl(socket_fd_, SIOCGIFINDEX, &interface_request) < 0) {
    const auto reason = errno_reason("ioctl(SIOCGIFINDEX)", errno);
    close_socket();
    return fail_result(kFaultTxFailure, TransportErrorClass::TxFailure, reason);
  }

  struct sockaddr_can address {};
  address.can_family = AF_CAN;
  address.can_ifindex = interface_request.ifr_ifindex;
  if (::bind(socket_fd_, reinterpret_cast<struct sockaddr *>(&address), sizeof(address)) < 0) {
    const auto reason = errno_reason("bind", errno);
    close_socket();
    return fail_result(kFaultTxFailure, TransportErrorClass::TxFailure, reason);
  }

  health_.connected = true;
  health_.writable = true;
  health_.heartbeat_ok = true;
  return BackendResult{true, "", 0U, 0.0};
}

BackendResult SocketCanBackend::send_native_frame(const struct can_frame & frame)
{
  if (socket_fd_ < 0 || !health_.connected || !health_.writable) {
    return fail_result(kFaultTxFailure, TransportErrorClass::TxFailure, "transport_closed");
  }

  const auto written = ::write(socket_fd_, &frame, sizeof(frame));
  if (written < 0) {
    return fail_result(
      kFaultTxFailure, TransportErrorClass::TxFailure,
      errno_reason("write", errno));
  }
  if (written != static_cast<ssize_t>(sizeof(frame))) {
    return fail_result(kFaultTxFailure, TransportErrorClass::TxFailure, "partial_write");
  }
  ++health_.tx_count;
  health_.safe_stop_active = false;
  return BackendResult{true, "", 1U, 0.0};
}

BackendResult SocketCanBackend::send(const CanFrame & frame, double now_sec)
{
  if (!valid_time(now_sec)) {
    return fail_result(kFaultTxFailure, TransportErrorClass::TxFailure, "invalid_time");
  }
  if (socket_fd_ < 0 || !health_.connected || !health_.writable) {
    return fail_result(kFaultTxFailure, TransportErrorClass::TxFailure, "transport_closed");
  }
  struct can_frame native {};
  std::string reason;
  if (!to_native_frame(frame, native, reason)) {
    return fail_result(kFaultTxFailure, TransportErrorClass::TxFailure, reason);
  }
  return send_native_frame(native);
}

BackendResult SocketCanBackend::send_control_group(
  const std::array<CanFrame, 2> & frames, double now_sec)
{
  if (!valid_time(now_sec)) {
    return fail_result(kFaultTxFailure, TransportErrorClass::TxFailure, "invalid_time");
  }
  if (socket_fd_ < 0 || !health_.connected || !health_.writable) {
    return fail_result(kFaultTxFailure, TransportErrorClass::TxFailure, "transport_closed");
  }
  std::array<struct can_frame, 2> native{};
  for (std::size_t index = 0U; index < native.size(); ++index) {
    std::string reason;
    if (!to_native_frame(frames[index], native[index], reason)) {
      return fail_result(kFaultTxFailure, TransportErrorClass::TxFailure, reason);
    }
  }
  const auto group_start = std::chrono::steady_clock::now();
  const auto first = send_native_frame(native[0]);
  if (!first.ok) {
    auto result = first;
    result.group_elapsed_sec = std::chrono::duration<double>(
      std::chrono::steady_clock::now() - group_start).count();
    return result;
  }
  const auto second = send_native_frame(native[1]);
  const auto group_elapsed_sec = std::chrono::duration<double>(
    std::chrono::steady_clock::now() - group_start).count();
  if (!second.ok) {
    return BackendResult{false, second.reason, 1U, group_elapsed_sec};
  }
  return BackendResult{true, "", 2U, group_elapsed_sec};
}

std::vector<CanFrame> SocketCanBackend::poll(double now_sec)
{
  std::vector<CanFrame> frames;
  if (!valid_time(now_sec) || socket_fd_ < 0 || !health_.connected) {
    return frames;
  }

  while (true) {
    struct pollfd descriptor {socket_fd_, POLLIN, 0};
    const auto ready = ::poll(&descriptor, 1, 0);
    if (ready == 0) {
      break;
    }
    if (ready < 0) {
      if (errno == EINTR) {
        continue;
      }
      enter_fail_safe(
        kFaultTxFailure, TransportErrorClass::TxFailure,
        errno_reason("poll", errno), true);
      break;
    }
    if ((descriptor.revents & (POLLERR | POLLHUP | POLLNVAL)) != 0) {
      enter_fail_safe(kFaultTxFailure, TransportErrorClass::TxFailure, "poll_error", true);
      break;
    }

    struct can_frame native {};
    const auto received = ::read(socket_fd_, &native, sizeof(native));
    if (received < 0) {
      if (errno == EAGAIN || errno == EWOULDBLOCK) {
        break;
      }
      enter_fail_safe(
        kFaultTxFailure, TransportErrorClass::TxFailure,
        errno_reason("read", errno), true);
      break;
    }
    if (received != static_cast<ssize_t>(sizeof(native))) {
      enter_fail_safe(kFaultTxFailure, TransportErrorClass::TxFailure, "partial_read", true);
      break;
    }
    if ((native.can_id & CAN_ERR_FLAG) != 0U) {
      process_error_frame(native);
      continue;
    }

    if (
      (native.can_id & (CAN_EFF_FLAG | CAN_RTR_FLAG)) != 0U ||
      (native.can_id & CAN_SFF_MASK) != native.can_id ||
      native.can_dlc != CAN_MAX_DLEN)
    {
      enter_fail_safe(kFaultTxFailure, TransportErrorClass::TxFailure, "invalid_rx_frame", false);
      continue;
    }
    frames.push_back(from_native_frame(native));
    ++health_.rx_count;
  }
  return frames;
}

BackendHealth SocketCanBackend::health() const
{
  return health_;
}

BackendResult SocketCanBackend::reconnect(double now_sec)
{
  if (!valid_time(now_sec)) {
    return fail_result(kFaultTxFailure, TransportErrorClass::TxFailure, "invalid_time");
  }
  close();
  const auto result = open();
  if (result.ok) {
    ++health_.restart_count;
    return BackendResult{true, "reconnected", 0U, 0.0};
  }
  return result;
}

}  // namespace car_hardware_adapter

#endif  // __linux__
