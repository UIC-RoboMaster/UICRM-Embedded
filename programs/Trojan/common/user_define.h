#pragma once

#include <cstdint>

/* ===================================== 底盘 ====================================== */
// 底盘标定完成开关. 确认轮向、传动比、PID 等后设 true, false 时底盘被禁止运动
inline constexpr bool CHASSIS_CALIBRATED = false;

// 底盘运动电机, 顺序为 fl, fr, bl, br
inline constexpr uint16_t chassis_motor_id[4] = {1, 2, 4, 3};                       // CAN ID
inline constexpr uint16_t chassis_motor_tx_id = 0x200;                              // 电调控制报文
inline constexpr uint16_t chassis_motor_rx_id[4] = {0x201, 0x202, 0x204, 0x203};    // 电调反馈报文
inline constexpr float chassis_motor_transmission_ratio[4] = {0, 0, 0, 0};          // 各轮减速比
inline constexpr bool chassis_motor_inverted[4] = {false, false, false, false};     // 各轮目标方向是否反转; true 反向

// 底盘速度环 PID
inline constexpr float chassis_omega_kp = 2500;                                     // 速度环 Kp
inline constexpr float chassis_omega_ki = 3;                                        // 速度环 Ki
inline constexpr float chassis_omega_kd = 3;                                        // 速度环 Kd
inline constexpr float chassis_max_out = 0;                                         // 输出限幅
inline constexpr float chassis_max_iout = 0;                                        // 积分限幅

// 角速度上限 rad/s
inline constexpr float chassis_max_motor_speed = 2 * PI * 7;

// 失联/命令过期阈值 ms
inline constexpr uint32_t CHASSIS_COMMAND_TIMEOUT = 100;


/* ================================== CAN Bridge ================================== */
inline constexpr uint8_t CAN_BRIDGE_GIMBAL_ID = 0x51;                               // CAN Bridge 上板地址
inline constexpr uint8_t CAN_BRIDGE_CHASSIS_ID = 0x52;                              // CAN Bridge 下板地址
inline constexpr uint8_t CAN_BRIDGE_CHASSIS_XY = 0x70;                              // 底盘平移命令寄存器编号
inline constexpr uint8_t CAN_BRIDGE_CHASSIS_TURN = 0x71;                            // 底盘使能与旋转命令寄存器编号
inline constexpr uint8_t CAN_BRIDGE_CHASSIS_POWER = 0x72;                           // 功率限制寄存器编号
inline constexpr uint8_t CAN_BRIDGE_CHASSIS_CURRENT_POWER = 0x73;                   // 兼容旧协议的实际功率/缓冲能量寄存器


/* ===================================== 云台 ====================================== */
// 云台运动电机, 顺序为 yaw, pitch, fold
inline constexpr uint16_t gimbal_motor_id[4] = {1, 2, 3};                            // CAN ID
inline constexpr uint16_t gimbal_motor_tx_id[4] = {0x101, 0x102, 0x103};             // 电调控制报文
inline constexpr uint16_t gimbal_motor_rx_id[4] = {0x201, 0x202, 0x203};             // 电调反馈报文, Master ID

// 拨弹电机
inline constexpr uint16_t steering_motor_id = 4;                                      // 拨弹电机 CAN ID
inline constexpr uint16_t steering_motor_tx_id = 0x104;                               // 拨弹电机控制报文
inline constexpr uint16_t steering_motor_rx_id = 0x204;                               // 拨弹电机反馈报文, Master ID
inline constexpr float steering_motor_transmission_ratio = 0;                         // 拨弹电机输出轴 到 拨弹机构 的外部传动比
inline constexpr float steering_step_angle = 0;                                       // 拨弹机构单发转角 rad

// 摩擦轮, 顺序为 l, r
inline constexpr uint16_t flywheel_motor_id[2] = {5, 6};                              // CAN ID
inline constexpr uint16_t flywheel_motor_tx_id = 0x1FF;                               // 电调控制报文
inline constexpr uint16_t flywheel_motor_rx_id[2] = {0x205, 0x206};                   // 电调反馈报文

// 遥控任务每轮处理后的等待 tick
inline constexpr uint32_t REMOTE_OS_DELAY = 10;
// 云台任务每轮更新后的等待 tick
inline constexpr uint32_t GIMBAL_OS_DELAY = 10;
// 上板底盘任务/下板 Chassis 更新后的等待 tick
inline constexpr uint32_t CHASSIS_OS_DELAY = 10;

