# PID 自动调参闭环设计

日期：2026-08-31

## 目标

提供一个可重复运行的 `python tools/pid/autotune.py`，完成“设置参数 → 固定工况 → 采集 → 停止 → 指标 → 评分 → 下一候选”的闭环。默认使用确定性离线模型验证工具链；显式选择 CAN 后端时连接真实 STM32。软件层不绕过 MCU 安全限制。

## 现有生效链路

PID 位于 `BSP/chassis_control.c` 的 10 ms 控制任务，左右轮各一个 `PID_t`。当前增益在初始化函数中硬编码。编码器提供可信速度，`Motor_Drive` 的输出范围为有符号 `-1000..1000`。现有 CAN V1 没有 PID 参数命令，电流/电压也没有反馈字段。

## 架构

- `protocol.py`：V1 运动命令编解码，以及新增的维护态 PID 增益事务帧编解码。
- `transport.py`：`SimulationTransport` 与可选 `PythonCanTransport`，统一 `set_pid`、发送命令、采样反馈、STOP/ESTOP 接口。
- `analysis.py`：从样本计算 rise time、overshoot、settling time、steady-state error、IAE、ISE、振荡和综合 score；任何安全/通信/数据完整性异常都返回不可用结果。
- `experiment.py`：固定低速/高速阶跃、加速、减速（必要时反转）工况，维护 `current_pid`、`candidate_pid`、`best_pid`、`best_score` 和 `experiment_history`，每轮使用 `try/finally` 停车并恢复安全参数。
- `search.py`：有界、可复现的 random search + 局部邻域候选，不假装从语言模型推导精确增益。
- `autotune.py`：命令行入口、配置加载、JSON/CSV 报告输出。

## PID 维护扩展

新增不影响现有驱动帧的维护命令区：`0x123 CMD_PID_GAINS` 发送一组公共/左右轮 PID 增益的分片事务，`0x187 FB_PID_GAINS` 返回接受/拒绝和当前值。事务只能在新鲜全零 STOP、无 E-stop、无 latched fault、维护确认条件满足时提交；任何驱动态请求拒绝。参数为非负 Q8.8 定点值，输出限制仍由 MCU 固定为 `±1000`，不写 Flash。

## 数据与安全

每条记录至少包含 ISO 时间戳、相对时间、Kp/Ki/Kd、工况、target、actual、error、control_output；current/voltage/safety_state/safety_fault 如果反馈存在则记录，否则写 `null`。超过速度、输出、振荡、样本新鲜度或安全阈值时立即发送 STOP，标记 trial aborted，并恢复上一组安全参数。

## 验证

先用 Python 单元测试验证指标边界、评分排序、模拟器轨迹、候选确定性、停车恢复和 JSON/CSV 输出；再运行 `python tools/pid/autotune.py --transport sim --iterations 3`。MCU 侧用现有 C host-test 编译模式覆盖帧格式、维护态门控和运行时增益应用；最后运行全 Python 测试和 CMake Debug 构建。没有实机 CAN 适配器时不声称已完成实车调参。
