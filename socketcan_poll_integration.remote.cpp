#include "car_hardware_adapter/socketcan_backend.hpp"

#include <array>
#include <chrono>
#include <cstdint>
#include <iostream>
#include <thread>
#include <vector>

using car_hardware_adapter::CanFrame;
using car_hardware_adapter::SocketCanBackend;
using car_hardware_adapter::SocketCanConfig;

int main()
{
  SocketCanBackend backend(SocketCanConfig{"can0"});
  const auto opened = backend.open();
  if (!opened.ok) {
    std::cerr << "open failed: " << opened.reason << '\n';
    return 2;
  }

  const std::vector<std::uint8_t> safe_stop{1U, 0xF9U, 0x0DU, 0x08U, 0U, 0U, 0U, 0U};
  const std::array<CanFrame, 2> group{{
    CanFrame{0x120U, false, false, safe_stop},
    CanFrame{0x121U, false, false, safe_stop},
  }};
  const auto sent = backend.send_control_group(group, 0.0);
  if (!sent.ok) {
    std::cerr << "safe-stop send failed: " << sent.reason << '\n';
    return 3;
  }

  const auto deadline = std::chrono::steady_clock::now() + std::chrono::milliseconds(300);
  while (std::chrono::steady_clock::now() < deadline) {
    backend.poll(0.0);
    std::this_thread::sleep_for(std::chrono::milliseconds(5));
  }

  const auto health = backend.health();
  if (!health.connected || !health.writable) {
    std::cerr << "poll disconnected safe-stop session: " << health.last_error << '\n';
    return 1;
  }
  return 0;
}
