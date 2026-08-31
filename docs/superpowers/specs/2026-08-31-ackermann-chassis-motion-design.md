# R3X Ackermann 车架运动框架完整设计规格

## 1. 目标

在现有 STM32 底盘闭环、CAN 协议、后轮速度 PID、舵机 PWM、看门狗和安全停机机制之上，补齐 R3X Ackermann 车架的主机侧运动学层。

主机接收：

* 车体纵向速度 `speed_mm_s`
* 等效前轮转角 `steering_mrad`

主机负责：

1. 校验车辆几何参数和输入命令。
2. 对等效转角执行安全限幅。
3. 根据 Ackermann 几何计算左右后轮目标速度。
4. 在任一轮目标速度超过最大允许轮速时，对左右轮速度统一同比例缩放。
5. 输出实际执行速度、转角、曲率、控制半径、内外轮信息和限幅状态。
6. 使用现有 CAN 协议发送：

   * `CMD_STEERING (0x120)`
   * `CMD_REAR_WHEELS (0x121)`
7. 记录运动学输入、限幅结果、CAN 发送结果和安全状态。

STM32 继续负责：

* 舵机 PWM 输出。
* 后轮速度 PID。
* 电机方向控制。
* 硬件相关限幅。
* CAN command watchdog。
* 急停。
* 故障状态。
* 超时停车。
* 底层安全保护。

本阶段不得在 STM32 中重复实现 Ackermann 运动学。

本阶段的核心目标是建立：

> 可重复、可测试、可审计、故障行为确定的主机侧运动学和命令适配层。

## 1.1 本仓库实现决策

为消除下文建议项在实现阶段的歧义，本仓库冻结以下选择：

* `VehicleGeometry` 在静态构造或配置加载时校验字段，非法值抛出
  `ValueError`；`solve_ackermann()` 对运行期命令、错误类型的 geometry 和轮速
  上限返回不可发送的安全无效结果，不向调用方抛出输入类异常。
* 新模块保持 Python 3.9 兼容，使用 frozen `dataclass`、`Enum` 和
  `Optional[str]` 表达不可变结果与机器可判定状态。
* `config.json` 的当前 `max_wheel_speed_mm_s` 使用现有车轮离地闭环台架已经
  验证过的保守命令值 `600.0`。它不是最终地面最高安全速度；实车分阶段验收后
  才允许提高。协议的 `3000 mm/s` 只作为编解码绝对上限。
* 正常命令适配一次只消费同一个 `AckermannResult` 并按 steering、wheels 顺序
  发送。任一帧失败即返回 `safe_stop_requested=True`，不得记为 committed；实际
  safe-stop 双帧由独立接口使用新的 command sequence 发送。若该双帧仍失败，
  状态明确标记 `WATCHDOG_FALLBACK`，依赖 MCU 的 100 ms command watchdog。
* 本次自动执行止于纯软件测试、现有主机协议交叉验证和固件回归构建。舵机、
  电机及动态实车验收必须按第 56～63 节由现场人员分阶段执行，不会因软件测试
  通过而自动启动车辆。

---

# 2. 设计边界

## 2.1 主机负责

主机负责车辆几何和高层命令转换：

```text
车体速度 + 等效转角
        ↓
输入与车辆参数校验
        ↓
转角安全限幅
        ↓
Ackermann 曲率
        ↓
左右后轮速度
        ↓
统一轮速缩放
        ↓
CAN command adaptation
        ↓
STM32
```

## 2.2 STM32 负责

STM32 只执行经过主机计算的目标：

```text
CMD_STEERING
    ↓
舵机角度/PWM映射
    ↓
PB6 / TIM4_CH1

CMD_REAR_WHEELS
    ↓
左右后轮目标速度
    ↓
轮速 PID
    ↓
电机 PWM
```

同时 STM32 保留现有：

```text
watchdog
estop
fault handling
over-current protection
timeout stop
hardware output limit
```

## 2.3 明确禁止

不得：

* 在主机和 MCU 两侧各实现一套 Ackermann 几何。
* 在 MCU 再次根据 steering 计算后轮差速。
* 单独截断某一个后轮速度。
* 为达到标称 350 mm 转弯半径而突破 ±20°限位。
* 使用车体总长或车体总宽代替轴距或轮距。
* 假设 USBTTL + TJA1050 本身等价于 Linux CAN 控制器。
* 在主机软件中覆盖 MCU 急停、watchdog 或底层故障状态。

---

# 3. R3X 已冻结车辆参数

```text
轮胎直径          65.0 mm
轮胎半径          32.5 mm
轴距              141.7 mm
轮距              120.0 mm

车体总长          230.0 mm
车体总宽          145.2 mm
底板最大宽度      131.7 mm

最大等效转角      ±20°
协议最大转角      ±349 mrad

后轴中心最小控制半径
                  ≈ 389.3 mm

保守场地扫掠半径  ≈ 520 mm
```

其中：

```text
wheelbase = 141.7 mm
track     = 120.0 mm
```

是 Ackermann solver 的核心几何参数。

以下参数：

```text
body_length
body_width
baseplate_width
sweep_radius
```

仅作为机械记录和安全场地检查参数，不参与左右轮速度计算。

---

# 4. 转角定义

内部运动学统一使用：

```text
rad
```

CAN 协议边界使用：

```text
integer mrad
```

定义：

```python
MAX_STEERING_DEG = 20.0
MAX_STEERING_RAD = radians(20.0)
MAX_STEERING_MRAD_PROTOCOL = 349
```

注意：

```text
20° = 349.06585 mrad
```

因此：

* Python 内部允许使用精确 `20°`
* CAN 编码发送时量化为最大 `349 mrad`

不得在不同模块中混用：

```text
20°
349 mrad
0.349 rad
```

作为多个互相独立的配置值。

必须有且只有一个几何真实来源。

---

# 5. 坐标和符号约定

冻结以下符号。

## 5.1 纵向速度

```text
speed > 0
车辆前进

speed < 0
车辆倒车

speed = 0
车辆静止
```

## 5.2 转角

```text
steering > 0
左转

steering < 0
右转

steering = 0
直行
```

## 5.3 曲率

```text
kappa > 0
左转

kappa < 0
右转

kappa = 0
直行
```

计算：

```text
kappa = tan(delta) / L
```

其中：

```text
delta = 等效转角
L     = wheelbase
```

单位：

```text
kappa = 1/mm
```

---

# 6. Ackermann 后轮速度模型

对于后轴中心纵向速度：

```text
v
```

曲率：

```text
kappa
```

轮距：

```text
T
```

左右后轮目标速度：

```text
v_left  = v × (1 - kappa × T / 2)

v_right = v × (1 + kappa × T / 2)
```

其中：

```text
T = track_mm
```

---

# 7. 左右轮物理解释

## 7.1 左转

当：

```text
steering > 0
kappa > 0
```

则前进时：

```text
v_left < v_right
```

即：

```text
left = inner
right = outer
```

## 7.2 右转

当：

```text
steering < 0
kappa < 0
```

则：

```text
v_right < v_left
```

即：

```text
right = inner
left = outer
```

---

# 8. 倒车语义

倒车不得改变几何内外轮定义。

例如：

```text
speed < 0
steering > 0
```

此时：

```text
v_left < 0
v_right < 0

abs(v_left) < abs(v_right)
```

几何关系仍然是：

```text
left = inner
right = outer
```

不得因为车辆倒车而交换：

```text
inner_side
outer_side
```

注意车辆实际 yaw rate：

```text
yaw_rate = v × kappa
```

因此倒车时，即使 steering 符号不变，车辆航向变化方向会随 `v` 的符号改变。

这是正常 Ackermann 几何行为。

---

# 9. 转弯半径定义

后轴中心控制半径：

```text
R = 1 / kappa
```

需要同时提供：

```text
rear_axle_radius_signed_mm
rear_axle_radius_abs_mm
```

当：

```text
kappa > 0
```

则：

```text
radius_signed > 0
```

当：

```text
kappa < 0
```

则：

```text
radius_signed < 0
```

直行时：

```text
kappa = 0

radius_signed = inf
radius_abs = inf

inner_side = None
outer_side = None
```

禁止用：

```text
0
None
非常大的 magic number
```

表示直行半径。

---

# 10. 最大转角对应半径

使用：

```text
L = 141.7 mm
delta = 20°
```

得到：

```text
R = L / tan(delta)
```

约：

```text
389.3 mm
```

因此必须冻结：

> R3X 当前软件可控制的后轴中心最小半径约为 389 mm，而不是 350 mm。

“350 mm 最小转弯半径”当前没有可靠测量基准，不作为控制目标。

不得通过：

```text
扩大 steering limit
```

强行匹配该标称数据。

---

# 11. 零速打方向行为

必须允许：

```text
speed = 0
steering != 0
```

此时：

```text
left_wheel_speed = 0
right_wheel_speed = 0

steering 正常输出
```

这样可以支持：

* 舵机机械检查。
* 中位验证。
* 转向方向验证。
* 舵机端点测试。
* 静态标定。

不得因为：

```text
speed == 0
```

而强制 steering 回中。

安全停车和舵机回中是两个不同概念。

---

# 12. 轮速限制设计

配置：

```text
max_wheel_speed_mm_s
```

必须：

```text
> 0
finite
```

原始计算：

```python
left_raw
right_raw
```

取：

```python
peak = max(abs(left_raw), abs(right_raw))
```

若：

```text
peak <= max_wheel_speed
```

则：

```text
scale = 1.0
```

否则：

```text
scale = max_wheel_speed / peak
```

最终：

```text
left_applied  = left_raw  × scale
right_applied = right_raw × scale

applied_speed = requested_speed × scale
```

这样必须保持：

```text
left / right ratio unchanged
```

即：

> 降低整体速度，但保持曲率不变。

禁止：

```python
left = clamp(left)
right = clamp(right)
```

分别截断。

因为分别截断会改变：

```text
left/right ratio
```

从而改变实际车辆曲率。

---

# 13. requested 与 applied 必须分离

所有运动学结果必须区分：

```text
用户要求值
```

与：

```text
实际发送值
```

至少包含：

```text
requested_speed_mm_s
requested_steering_rad

applied_speed_mm_s
applied_steering_rad

left_wheel_speed_mm_s
right_wheel_speed_mm_s
```

例如：

```text
requested speed = 1000
```

计算得到：

```text
outer wheel = 1100
```

而：

```text
max wheel speed = 1000
```

则最终：

```text
wheel_speed_scale < 1
applied_speed < requested_speed
```

日志必须能清楚解释：

> 速度降低来自主机运动学限速，而不是 STM32 PID 跟踪失败。

---

# 14. 数据结构设计

建议目录：

```text
tools/
└── vehicle/
    ├── __init__.py
    ├── geometry.py
    ├── ackermann.py
    ├── command.py
    ├── config.py
    └── config.json
```

测试：

```text
tests/
└── vehicle/
    ├── test_geometry.py
    ├── test_ackermann.py
    └── test_command.py
```

如果项目已有既定 Python test 目录规范，应遵循现有结构，不为了本模块强行新建不同风格。

---

# 15. geometry.py

职责：

* 定义车辆几何结构。
* 定义默认 R3X 参数。
* 做几何合法性检查。
* 管理尺寸单位。
* 避免 diameter 和 radius 双配置漂移。

推荐：

```python
@dataclass(frozen=True)
class VehicleGeometry:
    wheelbase_mm: float
    track_mm: float
    tire_diameter_mm: float
    max_steering_rad: float

    @property
    def tire_radius_mm(self) -> float:
        return self.tire_diameter_mm / 2.0
```

默认：

```python
R3X_GEOMETRY = VehicleGeometry(
    wheelbase_mm=141.7,
    track_mm=120.0,
    tire_diameter_mm=65.0,
    max_steering_rad=math.radians(20.0),
)
```

不得同时独立保存：

```text
diameter = 65
radius = 32.5
```

作为两个独立配置字段。

radius 应由 diameter 计算。

---

# 16. 几何参数合法性

以下字段必须：

```text
finite
> 0
```

包括：

```text
wheelbase_mm
track_mm
tire_diameter_mm
max_steering_rad
```

同时：

```text
max_steering_rad < pi / 2
```

因为：

```text
tan(delta)
```

在 ±90°附近不适合本模型。

非法 geometry：

```text
solver 返回 invalid
```

或者：

```text
构造 geometry 时抛出 ValueError
```

二者选择一种并在项目内统一。

推荐：

* 静态配置创建时使用异常。
* runtime solver 输入错误返回安全结果。

---

# 17. AckermannResult 设计

推荐：

```python
@dataclass(frozen=True)
class AckermannResult:
    valid: bool

    requested_speed_mm_s: float
    requested_steering_rad: float

    applied_speed_mm_s: float
    applied_steering_rad: float

    curvature_per_mm: float

    rear_axle_radius_signed_mm: float
    rear_axle_radius_abs_mm: float

    left_wheel_speed_mm_s: float
    right_wheel_speed_mm_s: float

    inner_side: str | None
    outer_side: str | None

    wheel_speed_scale: float

    steering_limited: bool
    wheel_speed_limited: bool

    safety_status: str
    reason: str | None
```

可根据现有代码风格使用：

```text
Enum
Literal
dataclass
NamedTuple
```

但语义必须完整保留。

---

# 18. safety_status

建议不要只用：

```text
bool valid
```

必须提供机器可判断状态。

例如：

```python
class MotionSafetyStatus(Enum):
    OK = "ok"
    STEERING_LIMITED = "steering_limited"
    SPEED_LIMITED = "speed_limited"
    LIMITED = "limited"

    INVALID_INPUT = "invalid_input"
    INVALID_GEOMETRY = "invalid_geometry"
    INVALID_SPEED_LIMIT = "invalid_speed_limit"
```

注意：

```text
steering limited
wheel speed limited
```

不一定是 failure。

它们属于：

```text
safe degraded command
```

而以下属于：

```text
invalid
```

例如：

```text
NaN
Inf
非法 geometry
max speed <= 0
```

---

# 19. solve_ackermann 接口

核心纯函数：

```python
result = solve_ackermann(
    speed_mm_s=500.0,
    steering_mrad=200,
    geometry=R3X_GEOMETRY,
    max_wheel_speed_mm_s=1000.0,
)
```

建议函数边界直接接受：

```text
steering_mrad
```

因为这是主机命令/API/CAN 领域常用接口单位。

函数内部立即：

```text
mrad -> rad
```

之后全部使用 rad。

也可以提供：

```python
solve_ackermann_rad()
```

作为内部接口。

但不得让核心代码反复在：

```text
degree
rad
mrad
```

之间转换。

---

# 20. solve_ackermann 标准执行流程

必须按以下顺序执行：

```text
1. Validate speed
2. Validate steering
3. Validate geometry
4. Validate max wheel speed
5. Convert steering mrad -> rad
6. Clamp steering
7. Calculate curvature
8. Calculate turning radius
9. Calculate raw left/right wheel speeds
10. Calculate inner/outer side
11. Calculate wheel speed scaling
12. Scale both wheels
13. Calculate applied body speed
14. Build AckermannResult
```

---

# 21. 输入有限性检查

必须使用：

```python
math.isfinite()
```

拒绝：

```text
NaN
+Inf
-Inf
```

分别对：

```text
speed
steering
geometry
max wheel speed
```

进行测试。

非法输入不得进入：

```text
tan()
division
CAN encode
```

---

# 22. 非法输入结果

当 runtime 输入非法时：

```text
valid = False
```

并提供：

```text
reason
```

例如：

```text
"speed_not_finite"
"steering_not_finite"
"invalid_geometry"
"invalid_max_wheel_speed"
```

非法结果：

```text
不得作为正常运动命令发送
```

但 Ackermann solver 本身：

> 不发送 CAN，不负责停车。

solver 必须是纯函数。

---

# 23. command.py 职责

`command.py` 负责：

```text
AckermannResult
      ↓
CAN protocol values
      ↓
现有 tools/pid/protocol.py
```

不得把 Ackermann 数学复制进 command.py。

结构：

```text
ackermann.py
    只懂运动学

command.py
    只懂如何把运动学结果变成协议命令

protocol.py
    只懂 CAN frame encode/decode
```

---

# 24. CAN 帧

继续使用现有协议：

```text
CMD_STEERING     0x120
CMD_REAR_WHEELS  0x121
```

必须复用已有：

```text
tools/pid/protocol.py
```

中的 encoder。

禁止在：

```text
tools/vehicle/command.py
```

重新手写一份帧格式。

---

# 25. steering CAN 编码

内部：

```text
applied_steering_rad
```

在协议边界转换：

```python
steering_mrad = round(applied_steering_rad * 1000)
```

然后最终硬限制：

```text
[-349, +349]
```

确保数值量化误差不能突破协议限位。

---

# 26. CAN 双帧一致性问题

当前：

```text
CMD_STEERING
CMD_REAR_WHEELS
```

是两个独立 CAN frame。

它们不是原子命令。

因此必须明确：

某一个瞬间 MCU 可能看到：

```text
new steering + old wheels
```

或者：

```text
old steering + new wheels
```

本阶段接受这一边界，不修改底层协议。

---

# 27. 双帧发送规则

一次主机控制周期必须：

```text
solve once
```

得到一个不可变：

```text
AckermannResult
```

然后：

```text
AckermannResult
    ↓
build steering frame
    ↓
build wheels frame
```

两个 frame 必须来自：

> 同一次 solve 的同一份 result。

禁止：

```text
send steering
重新读取命令
重新 solve
send wheels
```

---

# 28. 推荐发送顺序

正常控制周期：

```text
1. CMD_STEERING
2. CMD_REAR_WHEELS
```

原因是保持：

```text
先确定转向，再更新驱动力
```

作为统一协议行为。

这不是原子保证，只是确定的工程约定。

---

# 29. CAN 发送状态

一次 command cycle 至少区分：

```text
COMPUTE_OK
STEERING_TX_OK
WHEELS_TX_OK
COMMAND_COMMITTED
```

仅当两个 frame 都成功：

```text
COMMAND_COMMITTED = True
```

不得出现：

```text
steering 成功
wheels 失败
```

却记录成命令成功。

---

# 30. CAN 部分发送失败

例如：

```text
CMD_STEERING send success
CMD_REAR_WHEELS send failure
```

则整个控制周期：

```text
FAILED
```

立即进入：

```text
safe-stop request
```

并停止自动实验。

---

# 31. safe-stop 责任边界

safe-stop 不属于 Ackermann solver。

推荐：

```text
Ackermann solver
    ↓
Command adapter
    ↓
Transport/controller
    ↓
Safety handling
```

如果：

```text
result.valid == False
```

则：

```text
禁止发送正常运动 command
```

上层 controller 请求：

```text
rear wheel target = 0
```

舵机策略根据场景明确处理。

---

# 32. 默认安全停车命令

普通运行故障时，建议：

```text
rear wheels = 0
steering = 0
```

即：

```text
停车 + 回中
```

但注意：

静态舵机标定模式下：

```text
speed = 0
steering != 0
```

属于合法命令，不应被 safe-stop 逻辑误杀。

因此应区分：

```text
合法的零速转向
```

和：

```text
故障停车
```

---

# 33. CAN 总线本身失效

如果正常 command 发送失败：

上层可以尝试发送：

```text
steering = 0
rear wheels = 0
```

但如果：

```text
CAN interface
CAN controller
bus
adapter
```

本身已经不可用，则主机不能声称：

```text
safe stop sent successfully
```

此时必须记录：

```text
SAFE_STOP_TX_FAILED
WATCHDOG_FALLBACK
```

并依赖：

```text
STM32 command watchdog
```

最终进入安全停车。

---

# 34. 自动实验故障策略

如果发生任一：

```text
CAN send failure
feedback timeout
MCU safety stop
over-current
wheel error diverging
sustained oscillation
diagnostic fault
```

则：

```text
停止当前自动实验
停止参数继续搜索
请求安全停车
```

如果当前项目带自动 PID 调参：

```text
恢复最近一次已验证安全 PID
```

Ackermann 模块只报告状态。

不得：

```text
覆盖 MCU safety state
自动解除 estop
无条件重试高风险 command
```

---

# 35. config.json

配置文件可以保存：

```json
{
  "wheelbase_mm": 141.7,
  "track_mm": 120.0,
  "tire_diameter_mm": 65.0,
  "max_steering_deg": 20.0,
  "max_wheel_speed_mm_s": 1000.0
}
```

具体最大轮速使用项目现有已验证值。

如果尚未实测：

不得虚构一个最终工程参数。

可以：

```text
配置必须显式填写
```

或者：

```text
使用当前已有底盘安全上限
```

---

# 36. 不进入 config.json 的参数

当前不得保存未经实测的数据：

```text
servo center pulse
left endpoint pulse
right endpoint pulse
servo mechanical offset
left/right steering linkage ratio
actual inner front wheel angle
actual outer front wheel angle
```

这些必须等后续：

```text
机械标定
```

完成后再加入。

---

# 37. 日志设计

每次主机命令至少记录：

```text
timestamp

requested_speed_mm_s
requested_steering_mrad

applied_speed_mm_s
applied_steering_mrad

curvature_per_mm

rear_axle_radius_mm

left_target_mm_s
right_target_mm_s

inner_side
outer_side

steering_limited
wheel_speed_limited
wheel_speed_scale

result_valid
safety_status
reason

steering_tx_ok
wheels_tx_ok
command_committed
```

---

# 38. MCU feedback 日志

继续保存已有底层：

```text
left target
left actual
left error

right target
right actual
right error

PID output

controller state
diagnostic state
watchdog state
estop state
fault flags
```

从而以后能区分：

```text
运动学问题
PID问题
机械问题
通信问题
底层安全触发
```

---

# 39. 测试原则

必须：

> 先写测试，再实现功能。

核心 Ackermann solver 不依赖硬件。

测试必须能直接：

```bash
pytest
```

或加入当前项目已有 Python test runner。

---

# 40. geometry 单元测试

至少覆盖：

## G1

```text
diameter = 65.0
```

得到：

```text
radius = 32.5
```

## G2

默认：

```text
wheelbase = 141.7
track = 120.0
```

## G3

最大转角：

```text
20°
```

内部 rad 正确。

## G4

非法：

```text
wheelbase <= 0
track <= 0
diameter <= 0
```

必须拒绝。

## G5

```text
NaN
Inf
```

必须拒绝。

---

# 41. Ackermann 直行测试

输入：

```text
speed = 500
steering = 0
```

预期：

```text
kappa = 0

left = 500
right = 500

radius = inf

inner_side = None
outer_side = None

scale = 1
```

---

# 42. 左转测试

输入：

```text
speed > 0
steering > 0
```

必须：

```text
left < right
inner = left
outer = right
kappa > 0
```

---

# 43. 右转测试

输入：

```text
speed > 0
steering < 0
```

必须：

```text
right < left
inner = right
outer = left
kappa < 0
```

---

# 44. 倒车测试

输入：

```text
speed < 0
steering > 0
```

必须：

```text
left < 0
right < 0

abs(left) < abs(right)

inner = left
outer = right
```

确保代码没有因为倒车交换 inner/outer。

---

# 45. 最大转角测试

输入：

```text
steering = +349 mrad
steering = -349 mrad
```

应接受。

输入：

```text
steering = +500 mrad
steering = -500 mrad
```

应分别限制到：

```text
+20°
-20°
```

同时：

```text
steering_limited = True
```

---

# 46. 最小半径测试

最大转角时：

```text
abs(radius)
```

应约：

```text
389.3 mm
```

允许合理浮点误差。

必须明确测试：

```text
不是 350 mm
```

避免未来开发人员为了匹配宣传参数修改限位。

---

# 47. 轮速缩放测试

构造一个外轮超速情况。

记录缩放前：

```text
left_raw
right_raw
```

缩放后：

```text
left
right
```

验证：

```text
max(abs(left), abs(right))
<= max_wheel_speed
```

并验证：

```text
left_raw / right_raw
≈
left / right
```

曲率保持不变。

---

# 48. applied speed 测试

如果：

```text
scale < 1
```

则：

```text
applied_speed
=
requested_speed × scale
```

必须准确记录。

---

# 49. 非法输入测试

分别测试：

```text
speed = NaN
speed = +Inf
speed = -Inf

steering = NaN
steering = +Inf
steering = -Inf
```

必须：

```text
valid = False
```

不得产生正常运动 CAN frame。

---

# 50. 非法最大轮速测试

测试：

```text
max_wheel_speed = 0
max_wheel_speed < 0
NaN
Inf
```

必须拒绝。

---

# 51. 零速打方向测试

输入：

```text
speed = 0
steering = 200 mrad
```

预期：

```text
left = 0
right = 0

steering preserved
valid = True
```

command adapter 应可以发送：

```text
steering frame
wheel zero frame
```

---

# 52. CAN 编码兼容测试

对：

```text
CMD_STEERING
CMD_REAR_WHEELS
```

验证：

```text
command.py
```

最终调用现有：

```text
protocol.py
```

生成的 payload，与 STM32 当前解析格式兼容。

不得仅测试 Python 内部 encode/decode 自洽。

应根据：

```text
docs/CAN_PROTOCOL.md
STM32 decode implementation
```

交叉验证：

* 字节序。
* signed/unsigned。
* 数值单位。
* 位宽。
* 缩放比例。
* CAN ID。

---

# 53. CAN 双帧失败测试

至少模拟：

```text
steering success
wheels failure
```

验证：

```text
command_committed = False
```

并触发：

```text
safe stop request
```

另测试：

```text
steering failure
```

此时不得继续把 cycle 视为成功。

---

# 54. 全量 Python 回归

新增模块完成后必须运行：

```text
vehicle tests
+
existing PID tests
+
protocol tests
+
current full Python regression
```

不得只跑新增测试。

---

# 55. 测试数值容差

禁止：

```text
float ==
```

比较曲率或半径。

使用：

```python
math.isclose()
```

或：

```python
pytest.approx()
```

并设置合理：

```text
relative tolerance
absolute tolerance
```

---

# 56. 第一阶段纯软件验收

必须首先完成：

```text
geometry tests
ackermann tests
command adapter tests
CAN encode tests
```

此阶段：

```text
不连接电机
不跑舵机
```

---

# 57. 第二阶段 CAN 回环验收

验证：

```text
主机 encode
      ↓
CAN transport
      ↓
STM32/mock decode
```

检查：

```text
steering 数值
left speed
right speed
sign
byte order
unit
```

此阶段可以使用：

```text
mock MCU
CAN loopback
虚拟 CAN
```

优先避免直接实车。

---

# 58. 第三阶段舵机验收

车辆：

```text
车轮离地
```

首先确认：

```text
TIM4_CH1
PB6
PWM period
PWM pulse
```

然后：

```text
steering = 0
```

确认机械中位。

随后只测试小角度：

```text
±5°
```

再：

```text
±10°
```

最后：

```text
±20°
```

任何：

```text
机械卡死
高频抖动
异常电流
连杆干涉
PWM方向错误
```

立即停止。

---

# 59. 第四阶段低速直行

仅在舵机验证通过后。

命令：

```text
steering = 0
```

速度逐级：

```text
低速
中低速
```

验证：

```text
left/right target
actual
error
```

同时观察车辆是否：

```text
明显跑偏
振荡
PID不稳定
```

---

# 60. 第五阶段低速转弯

先：

```text
5°
```

再：

```text
10°
```

最后再考虑：

```text
20°
```

验证：

```text
左转时 left slower
右转时 right slower
```

并检查车辆运动方向是否符合软件约定。

---

# 61. 第六阶段动态工况

依次验证：

```text
加速
减速
低速阶跃
转弯阶跃
正转
必要时反转
```

严禁一开始进行：

```text
高速满舵
快速正反切换
极限速度 + 极限转角
```

---

# 62. 场地安全包络

当前记录：

```text
conservative sweep radius ≈ 520 mm
```

仅作为：

```text
实验场地安全提示
```

不得作为：

```text
精确碰撞模型
```

也不得替代：

```text
实体急停
机械限位
人员监护
真实防撞
```

---

# 63. 后续标定接口预留

当前 Ackermann 几何使用：

```text
equivalent steering angle
```

未来实车标定后可以扩展：

```text
equivalent angle
        ↓
servo mapping
        ↓
actual front-wheel geometry
```

但当前不得提前实现未经测量的复杂模型。

未来可加入：

```text
servo center
servo direction
servo endpoint
left/right asymmetry
linkage nonlinearity
actual left front angle
actual right front angle
```

---

# 64. 本阶段明确不解决

不解决：

1. 前悬、后悬精确测量。
2. 前轮实际转角测量。
3. 舵机连杆非线性。
4. PWM 中位机械标定。
5. PWM 左右端点精确标定。
6. 舵机左右非对称。
7. 轮胎侧偏。
8. 滑移。
9. 复杂车辆动力学。
10. 低速编码器量化。
11. 电机 deadzone。
12. 轮胎有效半径动态变化。
13. 350 mm 标称转弯半径反推。
14. 高速车辆稳定性控制。
15. 完整碰撞模型。
16. 自动路径规划。
17. ROS/Nav2 Ackermann controller 集成。

---

# 65. 最终文件职责

## geometry.py

负责：

```text
VehicleGeometry
R3X_GEOMETRY
geometry validation
unit conversion
```

## ackermann.py

负责：

```text
AckermannResult
solve_ackermann()
纯运动学
限幅
轮速缩放
```

不得依赖：

```text
CAN
serial
hardware
STM32
```

## command.py

负责：

```text
AckermannResult
      ↓
protocol command
```

处理：

```text
CAN values
CAN frame creation
双帧 command cycle
发送结果
safe-stop request
```

## protocol.py

继续负责：

```text
CAN frame encode/decode
```

不得复制。

## config.json

负责：

```text
可审计车辆参数
安全限值
```

---

# 66. 最终核心数据流

```text
requested speed
requested steering
        │
        ▼
input validation
        │
        ▼
steering clamp ±20°
        │
        ▼
kappa = tan(delta) / wheelbase
        │
        ▼
left/right rear wheel speed
        │
        ▼
uniform wheel speed scaling
        │
        ▼
AckermannResult
        │
        ├─────────────── log
        │
        ▼
command adapter
        │
        ├── CMD_STEERING 0x120
        │
        └── CMD_REAR_WHEELS 0x121
        │
        ▼
CAN transport
        │
        ▼
STM32
        │
        ├── steering PWM
        ├── rear wheel PID
        ├── watchdog
        ├── estop
        └── fault protection
```

---

# 67. 核心安全原则

必须始终满足：

```text
安全限幅优先于标称性能
```

```text
主机运动学不能绕过 MCU 安全机制
```

```text
非法输入不得形成正常运动命令
```

```text
左右轮超速必须统一缩放
```

```text
双 CAN frame 部分发送不得视为成功
```

```text
CAN 完全失效时依赖 MCU watchdog
```

```text
350 mm 不作为强制控制目标
```

```text
±20° 是当前不可突破的 steering 安全边界
```

---

# 68. 实现原则

AI 在实现时必须遵循：

1. 先检查现有：

   * `docs/CAN_PROTOCOL.md`
   * `tools/pid/protocol.py`
   * `BSP/chassis_control.c`
   * 当前测试结构。

2. 不猜测协议格式。

3. 不复制已有 protocol encoder。

4. 先写测试。

5. 再实现最小功能。

6. 每完成一个模块运行对应测试。

7. 最后运行全量回归。

8. 不修改无关模块。

9. 不修改 STM32 安全机制。

10. 如果发现当前设计和仓库真实协议不一致：

    * 以仓库现有冻结协议为事实。
    * 明确指出冲突。
    * 只修改主机适配层。
    * 不自行重构 CAN 协议。

---

# 69. 最终验收标准

本阶段只有在以下条件全部满足后才算完成：

* R3X 几何参数集中管理。
* diameter/radius 不存在双配置漂移。
* Ackermann solver 为纯函数。
* 直行左右轮相同。
* 左右转轮速差方向正确。
* 倒车符号正确。
* inner/outer 语义不因倒车改变。
* ±20°限幅正确。
* 最大转角半径约 389.3 mm。
* 超速时左右轮同比缩放。
* requested/applied 明确区分。
* 零速打方向被支持。
* NaN/Inf 有确定安全结果。
* 非法输入不会产生正常运动 CAN frame。
* CAN 编码与 STM32 当前协议兼容。
* 两帧使用同一份 AckermannResult。
* 部分 CAN 发送失败不能视为 command committed。
* CAN 全失效时正确进入 watchdog fallback 语义。
* 所有新增测试通过。
* 项目原有 Python 回归测试不退化。
* 实车测试严格按照：
  `纯软件 → CAN回环 → 轮离地 → 舵机 → 低速直行 → 低速转弯 → 动态工况`
  的顺序执行。

在以上条件满足前，不继续扩展更复杂的车辆动力学、自动导航或转向机械补偿。
