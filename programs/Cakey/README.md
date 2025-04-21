# 某台垃圾步兵

## 程序指南

本程序为步兵的云台程序，底盘程序直接使用 `example_chassis_DJI_Board_TypeC_general_can_bridge`。

## 接线相关

云台上开发板是一块 RoboMaster 开发板A，使用的电机如下：
- pitch电机：GM6020 can2+0x20A 电流控制模式
- yaw电机：GM6020 can1+0x209 电流控制模式
- 拨弹电机：M2006 can1+0x207 使用老的ServoMotor类
- 摩擦轮电机：M2305 PWM 定时器1+通道1/通道4
- 底盘：通过 Can Bridge 连接控制4个底盘3508电机

使用的其他接口如下：
- UART8：调试接口
- UART6：~~外置IMU~~后续更换为mini-PC对接接口
- UART1：D-Bus接收机
- UART3：裁判系统串口
- UART7：图传串口，用于接收图传数据