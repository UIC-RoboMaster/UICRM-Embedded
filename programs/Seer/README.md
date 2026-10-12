# Navigator 2026 哨兵 Seer/断头龙 

大小 yaw 全向轮的哨兵 Developed by Martin 2026-2027

## 电路相关
Navigator 2026 哨兵 Seer/断头龙 采用 双 DJI Robomaster C型开发板

- 底盘：4个 3508电机 底盘 can2
    - fl 0x204
    - fr 0x201
    - bl 0x203
    - br 0x202

- 大yaw：GM6020 canbridge can1 id4 = 0x208
  - 正向安装，逆时针旋转为正，电机角度++
- 小yaw：GM6020 云台 can2 id2 = 0x206
  - 正向安装，逆时针旋转为正，电机角度++
- pitch：GM6020 云台 can2 id6 = 0x20a
  - 侧向安装，以抬头为正方向，电机角度++

- 拨弹：M2006 云台 can2 0x203
  - 正向安装
- 摩擦轮电机：M3508 * 2
    - 左摩擦轮 云台 can2 0x202
    - 右摩擦轮 云台 can2 0x201

使用的其他接口如下：
- 底盘 UART6 ：
- 底盘 UART1 ：调试接口
- 云台 UART6 ：
- 云台 UART1 ： 调试接口
- 底盘 / 云台UART3：dbus接收机
- 云台 UART：裁判系统串口（暂无）
- 云台 UART：图传串口，用于接收图传数据（暂无）

- IMU 正对安装
  - INS_angle[0] 为 yaw 轴角度
  - INS_angle[1] 为 pitch 轴角度
  - INS_angle[2] 暂时未使用

## 程序指南

### 底盘 chasiss
chassis_example.cpp 为独立的底盘 example，直接用底部c板控制4个3508电机，并可以使用DBUS遥控器。

### 大小 yaw 云台 dual_yaw_gimbal
组件：`boards/components/include/dual_yaw_gimbal.h` + `boards/components/src/dual_yaw_gimbal.cpp`

#### 机构与控制思路

同轴串联两级：底盘 → 大yaw(下) → 小yaw(上) → 枪口

因此

    云台相对地面的朝向 = 下yaw关节角 + 上yaw关节角

- **小yaw**：主控快轴。补上下yaw还没到位的部分，但被夹在自己 ±90° 的行程内；
- **大yaw**：同周期直接接手完整角度需求，快慢由电机位置/速度环决定。
- 小yaw不提前让出大yaw尚未完成的角度，而是根据实际反馈持续补偿，最终回中。

例：目标相对车身 +120° —— 下yaw一开始就朝 +120° 转（慢），上yaw先快速转到 +90°（夹限位）；
随着下yaw转过 30°，上yaw的目标从 +90° 连续回缩到 0°，稳态为 **下yaw = +120°、上yaw = 0°**。

注意：`upper_yaw_max_` 是**上yaw关节**相对下yaw的行程，不是云台朝向的限幅，
`TargetAbs()` 的 yaw 目标可以给任意角度，怎么分给两个电机由协调逻辑决定。

#### 待办
- pid：大小yaw轴/pitch轴pid问题
- 小陀螺：下yaw需要叠加底盘旋转速度前馈（参考 `programs/Sentry/gimbal/src/gimbal_task.cpp`）。

