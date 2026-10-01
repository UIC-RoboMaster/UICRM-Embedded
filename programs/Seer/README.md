# Navigator 2026 哨兵 Seer/断头龙

双yaw / 大小yaw 构型的全向轮 哨兵

## 电路相关
Navigator 2026 哨兵 Seer/断头龙 采用 双 DJI Robomaster C型开发板

- 底盘：4个 3508电机 底盘 can2
    - fl 0x204
    - fr 0x201
    - bl 0x203
    - br 0x202

- 大yaw：GM6020 canbridge can1 id4 = 0x208
- 小yaw：GM6020 云台 can2 id2 = 0x206
- pitch：GM6020 云台 can2 id6 = 0x20a

- 拨弹：M2006 云台 can2 0x203
- 摩擦轮电机：M3508 * 2
    - 左摩擦轮 云台 can2 0x202
    - 右摩擦轮 云台 can2 0x201

使用的其他接口如下：
- 底盘 UART6 ：
- 底盘 UART1 ：调试接口
- 云台 UART6 ：
- 云台 UART1 ： 调试接口

- UART6：~~外置IMU~~后续更换为mini-PC对接接口
- UART1：D-Bus接收机
- UART3：裁判系统串口
- UART7：图传串口，用于接收图传数据

## 程序指南

### 底盘 chasiss
chassis_example.cpp 为独立的底盘 example，直接用底部c板控制4个3508电机，并可以使用DBUS遥控器。







### 双 yaw 云台 dual_yaw_gimbal

组件：`boards/components/include/dual_yaw_gimbal.h` + `boards/components/src/dual_yaw_gimbal.cpp`

#### 机构与控制思路

同轴串联两级：底盘 → 大yaw(下) → 小yaw(上) → 枪口，因此

    云台相对地面的朝向 = 下yaw关节角 + 上yaw关节角

- **上yaw(小yaw) 轻、快**：主控快轴。补上下yaw还没到位的部分，但被夹在自己 ±90° 的行程内；
- **下yaw(大yaw) 重、慢**：慢轴。从第一周期起就朝最终由它承担的朝向走，每周期按
  `lower_yaw_recenter_max_step` 限速地接手；上yaw同步补差额。
- 两个yaw**从一开始都在转**：上yaw快、下yaw慢，下yaw逐步把角度接过手，上yaw随之收回中心。
  两者之和 = 本周期朝向误差，所以过程中朝向仍由 IMU 闭环保证。
  最终上yaw回中心、下yaw承担全部角度，上yaw重新拿到左右各90°的快速权限。

例：目标相对车身 +120° —— 下yaw一开始就朝 +120° 转（慢），上yaw先快速转到 +90°（夹限位）；
随着下yaw转过 30°，上yaw的目标从 +90° 连续回缩到 0°，稳态为 **下yaw = +120°、上yaw = 0°**。

注意：`upper_yaw_max_` 是**上yaw关节**相对下yaw的行程，不是云台朝向的限幅，
`TargetAbs()` 的 yaw 目标可以给任意角度，怎么分给两个电机由协调逻辑决定。

IMU 必须装在云台(上yaw输出)上，此时 `imu_yaw` 就是枪口相对地面的朝向，
`UpdateIMU()` 那个增量闭环才有稳定点；IMU 在底盘上会每周期都被命令再转一个固定角（跑飞）。

#### 参数与标定

| 参数 | 含义 |
| --- | --- |
| `upper_yaw_max_` | 上yaw关节相对下yaw的机械行程，本车 90° = PI/2 |
| `upper_yaw_circle_` | 上yaw能否整圈，本车 false |
| `lower_yaw_circle_` | 下yaw能否连续旋转，本车 true（有限位时置 false 并给 `lower_yaw_max_`） |
| `upper_yaw_offset_` | 上yaw关节处在正中（相对下yaw为0）时的编码器读数 |
| `lower_yaw_offset_` | 下yaw指向车身正前方时的编码器读数 |
| `upper_yaw_joint_inverted` | 上yaw编码器增大时，云台地面朝向是否同向增大，反向则置 true |
| `lower_yaw_joint_inverted` | 同上，下yaw |
| `lower_yaw_recenter_max_step` | 下yaw每周期接手/回中的步长上限 [rad]，等效交接角速度 ≈ 该值 / 下yaw位置环时间常数。0 = 下yaw不主动接手（只补上yaw顶限位的差额） |

标定顺序建议：

1. 6020 记得 `SetTransmissionRatio(1)`；编码器读数和 offset 都在"DjiMotorBase 输出轴累计角"
   （`GetOutputShaftCumulatedTheta()`，上电位置为 0）这一域，标定时读这个值。
   把上yaw转到相对下yaw的正中，读数记作 `upper_yaw_offset_`；下yaw指向车身正前方时读数记作
   `lower_yaw_offset_`（上电相关，所以每次上电都要标）。
2. `upper_yaw_joint_inverted` / `lower_yaw_joint_inverted`：给一个小角度目标，看云台实际往哪边转，
   反向就置 true。标错的典型现象是上yaw顶在限位、朝向一直追不上目标（有界，不会跑飞，但明显不对）。
3. `lower_yaw_recenter_max_step` 先给 0（下yaw不主动接手）：确认 ±90° 内上yaw单独跟踪、
   超过90°后下yaw接手。再给一个正值（如 1°~2°）让下yaw从第一周期起就接手、上yaw回中心。
   交接越快，上yaw已经让出角度而下yaw还没跟上的短暂朝向偏差越大（量级约等于该步长），
   按现场精度要求折中。

#### 待办

- `programs/Seer/gimbal` 还是空 target：CAN 初始化、遥控/自瞄输入、`UpdateIMU()` 调用还没写。
- 小陀螺：下yaw需要叠加底盘旋转速度前馈（参考 `programs/Sentry/gimbal/src/gimbal_task.cpp`）。

