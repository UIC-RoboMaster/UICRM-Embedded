/*###########################################################
 # Copyright (c) 2026-2027. BNU-HKBU UIC RoboMaster         #
 #                                                          #
 # This program is free software: you can redistribute it   #
 # and/or modify it under the terms of the GNU General      #
 # Public License as published by the Free Software         #
 # Foundation, either version 3 of the License, or (at      #
 # your option) any later version.                          #
 #                                                          #
 # This program is distributed in the hope that it will be  #
 # useful, but WITHOUT ANY WARRANTY; without even           #
 # the implied warranty of MERCHANTABILITY or FITNESS       #
 # FOR A PARTICULAR PURPOSE.  See the GNU General           #
 # Public License for more details.                         #
 #                                                          #
 # You should have received a copy of the GNU General       #
 # Public License along with this program.  If not, see     #
 # <https://www.gnu.org/licenses/>.                         #
 ###########################################################*/

#pragma once
#include "gimbal_DoubleYaw.h"
#include "pid.h"

// namespace seer {
     // Smooth integer-rpm feedback; 1.0f bypasses filtering.
     // constexpr float kPitchSpeedFeedbackFilter = 0.5f;
// }

// basic information of gimbal
// 零点、关节方向和电机闭环响应需要按本车实测标定，不能直接沿用示例零点。
const control::gimbal_DoubleYaw_data_t gimbal_init_data = {
    .pitch_offset_ = 4.35,
    .pitch_max_ = 1,
    .pitch_inverted = false,  // task 直接传入 pitch 目标/增量，不再两次反号。
    .pitch_deadband = 0,

    .upper_yaw_offset_ = 3.1,
    .upper_yaw_max_ = PI / 2,
    .upper_yaw_circle_ = false,
    .upper_yaw_inverted = false,  // 小 yaw 编码器增大时，IMU yaw 同向增大。
    .upper_yaw_deadband = 0,

    .lower_yaw_offset_ = 0.5f,
    .lower_yaw_max_ = PI,
    .lower_yaw_circle_ = true,
    .lower_yaw_inverted = false,  // 大 yaw 编码器增大时，IMU yaw 同向增大。
    .lower_yaw_deadband = 0,

};

// Pitch
// Allow low-speed integral to support the load; taper accumulation at large speed errors.
const control::ConstrainedPID::PID_Init_t pitch_omega_pid_init = {
    .kp = 800,
    .ki = 2,
    .kd = 0,
    .max_out = 8000,  // 最大电流输出，参考说明书
    .max_iout = 3000,
    .deadband = 0,                                                // 死区
    .A = 1.5f,  // Taper same-sign accumulation from 0.5 to 2 rad/s speed error.
    .B = 0.5f,
    .output_filtering_coefficient = 0.1,                          // 输出滤波系数
    .derivative_filtering_coefficient = 0,                        // 微分滤波系数
    .mode = control::ConstrainedPID::Integral_Limit |
            control::ConstrainedPID::OutputFilter |               // 输出滤波
            control::ConstrainedPID::ChangingIntegralRate
};

const control::ConstrainedPID::PID_Init_t pitch_theta_pid_init = {
    .kp = 18,
    .ki = 0,
    .kd = 0,
    .max_out = 5 * PI,  // Position-loop output is a speed command in rad/s.
    .max_iout = 0,
    .deadband = 0,                                 // 死区
    .A = 0,                                        // 变速积分所能达到的最大值为A+B
    .B = 0,                                        // 启动变速积分的死区
    .output_filtering_coefficient = 0.1,           // 输出滤波系数
    .derivative_filtering_coefficient = 0,         // 微分滤波系数
    .mode = control::ConstrainedPID::Integral_Limit |
            control::ConstrainedPID::OutputFilter                // 输出滤波
};


// Yaw output filtering is disabled while tuning the angle/speed loops.
// For this PID, coefficient 1 passes the current output; 0 retains the old output.
// 配对调整角度/速度 Kp：保持两者乘积，增加速度反馈的制动增益。
// 小 yaw
const control::ConstrainedPID::PID_Init_t upper_yaw_omega_pid_init = {
    .kp = 700,  // 与角度环 Kp=5 配对，增强速度制动。
    .ki = 2,
    .kd = 0,
    .max_out = 4000,
    .max_iout = 800,
    .deadband = 0,                                          // 死区
    .A = 1.5 * PI,                                          // 变速积分所能达到的最大值为A+B
    .B = 1 * PI,                                            // 启动变速积分的死区
    .output_filtering_coefficient = 0.1,                    // 不平滑输出
    .derivative_filtering_coefficient = 0,             // 微分滤波系数
    .mode = control::ConstrainedPID::Integral_Limit |       // 积分限幅
            control::ConstrainedPID::OutputFilter |               // 输出滤波
            control::ConstrainedPID::Trapezoid_Intergral |  // 梯形积分
            control::ConstrainedPID::ChangingIntegralRate,  // 变速积分
};

const control::ConstrainedPID::PID_Init_t upper_yaw_theta_pid_init = {
    .kp = 8,
    .ki = 0,
    .kd = 0,
    .max_out = 8 * PI,  // 角度环输出是 rad/s，速度补偿在此输出之后叠加。
    .max_iout = 1,
    .deadband = 0,                                 // 死区
    .A = 0,                                        // 变速积分所能达到的最大值为A+B
    .B = 0,                                        // 启动变速积分的死区
    .output_filtering_coefficient = 0.1,           // 不平滑输出
    .derivative_filtering_coefficient = 0,         // 微分滤波系数
    .mode = control::ConstrainedPID::Integral_Limit |
        control::ConstrainedPID::OutputFilter               // 输出滤波
};

// 大 yaw
const control::ConstrainedPID::PID_Init_t lower_yaw_omega_pid_init = {
    .kp = 1500,
    .ki = 1,
    .kd = 2000,
    .max_out = 7000,
    .max_iout = 1500,
    .deadband = PI / 180,
    .A = 1.5 * PI,
    .B = 1 * PI,
    .output_filtering_coefficient = 0.01,
    .derivative_filtering_coefficient = 0.05,
    .mode = control::ConstrainedPID::Integral_Limit |
            control::ConstrainedPID::OutputFilter |               // 输出滤波
            control::ConstrainedPID::Trapezoid_Intergral |
            control::ConstrainedPID::ChangingIntegralRate,
    .retain_integral_in_deadband = true,  // Keep learned holding output at zero speed error; Disable clears it.
};

const control::ConstrainedPID::PID_Init_t lower_yaw_theta_pid_init = {
    .kp = 2,
    .ki = 0,
    .kd = 800,
    .max_out = 5 * PI,
    .max_iout = 0,
    .deadband = PI / 180,
    .A = 0,
    .B = 0,
    .output_filtering_coefficient = 0.01,
    .derivative_filtering_coefficient = 0.001,
    .mode = control::ConstrainedPID::Integral_Limit |
            control::ConstrainedPID::OutputFilter                // 输出滤波

};
