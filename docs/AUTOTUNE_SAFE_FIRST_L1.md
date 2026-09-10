# AutotuneSafe 首轮 L1 真机操作边界

若启用 CAN autotune transport，本流程的 CAN V1 速率固定为 500 kbit/s；
本地 SWD session 路径不依赖 CAN，也不因该物理层修改而改变。

本流程只适用于软件验证已经通过、后轮架空且人工监督的首轮调参。它不是 H5 地面测试授权；H5 始终禁止。

## 刷写前

1. 确认后轮完全架空、机械无干涉，E-stop 在操作者手边，电源和 TB6612 状态正常。
2. 确认当前使用 `build/AutotuneSafe/ros.elf`（或同目录生成的 `.bin/.hex`），该 preset 未启用任何 bench 宏。
3. 通过 ST-LINK 将 ELF 从 `0x08000000` 下载并复位。软件阶段不会自动启动电机。
4. 上电后先观察 telemetry：应为 `L0/LOCKED` 或完成本地 preflight 后的 `L1/READY`，PWM 应为零；不要用普通 Debug 镜像做调参。

## 首轮运行

1. host 创建新的 `session_id` 和 `experiment_id`，加载 MCU 报告的 seed/best-known-safe PID；候选必须通过 MCU 的 Q8.8、绝对范围和 bootstrap step 检查。
2. 只发送正方向目标，目标不超过 `100 mm/s`。首轮使用编译期 L1 PWM hard limit `150‰`；不得请求直接反转、300 mm/s 或全 PWM。
3. 发送 `START` 后持续发送严格递增的 session heartbeat，并按 `0 → target ramp → 0` 运行。每个测试点结束后发送 `STOP`，等待两轮实际速度都低于 stillness threshold 并保持确认，再开始下一点。
4. 主机同时检查 CAN freshness、完整 snapshot sequence、`fault_code/safety_state` 和 abort 字段；MCU 的 stall、overspeed、oscillation、saturation、watchdog、E-stop/Safety 保护独立有效。
5. 任一 abort 发生时立即停止当前实验，不重复激进搜索同一区域；等待轮速归零，保存 `abort_reason` 和该轮 telemetry。参数不得进入 best-known-safe。
6. 首轮不得使用 Ziegler–Nichols 持续加 Kp 诱发振荡，也不得用一次高 PWM 冲击寻找临界增益。没有电流/温度传感器，不能把软件时间代理当作真实过流保护。

## 收尾

1. 发送 STOP，确认轮速归零并让系统完成 cooldown；保留实验 JSON/CSV 和 MCU snapshot。
2. 调参结束后重新刷写 `build/Debug/ros.elf`，复位并确认普通 Debug/Release 没有 CAN 可进入 AutotuneSafe actuator 模式。
3. L2 及以上 envelope 目前未配置、未验证，不能通过 `REQUEST_PROMOTE` 解锁；不进入 H5。
