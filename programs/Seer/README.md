# Navigator 2026 哨兵 Seer/断头龙

双yaw / 大小yaw 构型的全向轮 哨兵

## 电路相关
Navigator 2026 哨兵 Seer/断头龙 采用 双 DJI Robomaster C型开发板

- 大yaw电机：GM6020 0x204
- 小yaw电机：GM6020 0x205
- pitch电机：GM6020 0x206

- 拨弹电机：M2006
- 摩擦轮电机：M3508 * 2
    - 左摩擦轮
    - 右摩擦轮
- 底盘：通过 Can Bridge 连接控制4个底盘3508电机

使用的其他接口如下：
- UART8：调试接口
- UART6：~~外置IMU~~后续更换为mini-PC对接接口
- UART1：D-Bus接收机
- UART3：裁判系统串口
- UART7：图传串口，用于接收图传数据

## 程序指南

### 底盘 chasiss
chassis_example.cpp 为独立的底盘 example，直接用底部c板控制4个3508电机，并可以使用DBUS遥控器。



