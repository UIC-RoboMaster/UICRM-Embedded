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
#include "dual_yaw_gimbal.h"
#include "pid.h"

// basic information of gimbal
// 零点、关节方向和电机闭环响应需要按本车实测标定，不能直接沿用示例零点。
const control::dual_yaw_gimbal_data_t gimbal_init_data = {
    .pitch_offset_ = 1.8f,
    .pitch_max_ = 0.595F,
    .pitch_inverted = true,
    .pitch_deadband = 0,

    .upper_yaw_offset_ = 3.1,
    .upper_yaw_max_ = PI / 2,
    .upper_yaw_circle_ = false,
    .upper_yaw_inverted = false,
    .upper_yaw_deadband = 0,

    .lower_yaw_offset_ = 2.1f,
    .lower_yaw_max_ = PI,
    .lower_yaw_circle_ = true,
    .lower_yaw_inverted = false,
    .lower_yaw_deadband = 0,

    .upper_yaw_joint_inverted = false,
    .lower_yaw_joint_inverted = false,
};

// Pitch
const control::ConstrainedPID::PID_Init_t pitch_omega_pid_init = {
    .kp = 0,
    .ki = 0,
    .kd = 0,
    .max_out = 16384,  // 最大电流输出，参考说明书
    .max_iout = 4000,
    .deadband = 0,                                                // 死区
    .A = 1.5 * PI,                                                // 变速积分所能达到的最大值为A+B
    .B = 1 * PI,                                                  // 启动变速积分的死区
    .output_filtering_coefficient = 0.1,                          // 输出滤波系数
    .derivative_filtering_coefficient = 0,                        // 微分滤波系数
    .mode = control::ConstrainedPID::Integral_Limit |             // 积分限幅
            control::ConstrainedPID::OutputFilter |               // 输出滤波
            control::ConstrainedPID::Trapezoid_Intergral |        // 梯形积分
            control::ConstrainedPID::ChangingIntegralRate |       // 变速积分
            control::ConstrainedPID::Derivative_On_Measurement |  // 微分在测量值上
            control::ConstrainedPID::DerivativeFilter             // 微分在测量值上
};

const control::ConstrainedPID::PID_Init_t pitch_theta_pid_init = {
    .kp = 0,
    .ki = 0.005,
    .kd = 0,
    .max_out = 6 * PI,  // 最高旋转速度
    .max_iout = 0,
    .deadband = 0,                                 // 死区
    .A = 0,                                        // 变速积分所能达到的最大值为A+B
    .B = 0,                                        // 启动变速积分的死区
    .output_filtering_coefficient = 0.1,           // 输出滤波系数
    .derivative_filtering_coefficient = 0,         // 微分滤波系数
    .mode = control::ConstrainedPID::OutputFilter  // 输出滤波
};


// 小 yaw
const control::ConstrainedPID::PID_Init_t upper_yaw_omega_pid_init = {
    .kp = 300,
    .ki = 1,
    .kd = 0,
    .max_out = 16384,
    .max_iout = 2000,
    .deadband = 0,                                          // 死区
    .A = 1.5 * PI,                                          // 变速积分所能达到的最大值为A+B
    .B = 1 * PI,                                            // 启动变速积分的死区
    .output_filtering_coefficient = 1,                    // 输出滤波系数
    .derivative_filtering_coefficient = 0,             // 微分滤波系数
    .mode = control::ConstrainedPID::Integral_Limit |       // 积分限幅
            control::ConstrainedPID::OutputFilter |         // 输出滤波
            control::ConstrainedPID::Trapezoid_Intergral |  // 梯形积分
            control::ConstrainedPID::ChangingIntegralRate,  // 变速积分
};

const control::ConstrainedPID::PID_Init_t upper_yaw_theta_pid_init = {
    .kp = 10,
    .ki = 0,
    .kd = 0,
    .max_out = 16384,
    .max_iout = 0,
    .deadband = 0,                                 // 死区
    .A = 0,                                        // 变速积分所能达到的最大值为A+B
    .B = 0,                                        // 启动变速积分的死区
    .output_filtering_coefficient = 1,           // 输出滤波系数
    .derivative_filtering_coefficient = 0,         // 微分滤波系数
    .mode = control::ConstrainedPID::OutputFilter  // 输出滤波
};

// 大 yaw
const control::ConstrainedPID::PID_Init_t lower_yaw_theta_pid_init = {
    .kp = 2.0f,
    .ki = 0.0f,
    .kd = 0.0f,
    .max_out = 16384,
    .max_iout = 2000,
    .deadband = 0.0f,
    .A = 0,
    .B = 0,
    .output_filtering_coefficient = 1,
    .derivative_filtering_coefficient = 0.0f,
    .mode = control::ConstrainedPID::Integral_Limit
};

const control::ConstrainedPID::PID_Init_t lower_yaw_omega_pid_init = {
    .kp = 100,
    .ki = 1,
    .kd = 4,
    .max_out = 16384,
    .max_iout = 2000,
    .deadband = 0,
    .A = 1.5 * PI,
    .B = 1 * PI,
    .output_filtering_coefficient = 1,
    .derivative_filtering_coefficient = 0,
    .mode = control::ConstrainedPID::Integral_Limit |
            control::ConstrainedPID::OutputFilter |
            control::ConstrainedPID::Trapezoid_Intergral |
            control::ConstrainedPID::ChangingIntegralRate,
};
