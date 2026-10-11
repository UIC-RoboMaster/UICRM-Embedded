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
#include "pid.h"
#include "utils.h"

// left / right fric motor
const control::ConstrainedPID::PID_Init_t flywheel_omega_pid_init = {
    .kp = 150,
    .ki = 0.03,
    .kd = 1,
    .max_out = 30000,
    .max_iout = 10000,
    .deadband = 0,                                          // 死区
    .A = 3 * PI,                                            // 变速积分所能达到的最大值为A+B
    .B = 2 * PI,                                            // 启动变速积分的死区
    .output_filtering_coefficient = 0.1,                    // 输出滤波系数
    .derivative_filtering_coefficient = 0,                  // 微分滤波系数
    .mode = control::ConstrainedPID::Integral_Limit |       // 积分限幅
            control::ConstrainedPID::OutputFilter |         // 输出滤波
            control::ConstrainedPID::Trapezoid_Intergral |  // 梯形积分
            control::ConstrainedPID::ChangingIntegralRate,  // 变速积分
};

// steering motor
const control::ConstrainedPID::PID_Init_t steering_omega_pid_init = {
    .kp = 800,
    .ki = 0,
    .kd = 5000,
    .max_out = 10000,
    .max_iout = 0,
    .deadband = 0,                                           // 死区
    .A = 2 * PI,                                             // 变速积分所能达到的最大值为A+B
    .B = 1.5 * PI,                                           // 启动变速积分的死区
    .output_filtering_coefficient = 0.1,                     // 输出滤波系数
    .derivative_filtering_coefficient = 0,                   // 微分滤波系数
    .mode = control::ConstrainedPID::Integral_Limit |        // 积分限幅
            control::ConstrainedPID::OutputFilter |          // 输出滤波
            control::ConstrainedPID::Trapezoid_Intergral |   // 梯形积分
            control::ConstrainedPID::ChangingIntegralRate |  // 变速积分
            control::ConstrainedPID::ErrorHandle,            // 错误处理

};

const control::ConstrainedPID::PID_Init_t steering_theta_pid_init = {
    .kp = 30,
    .ki = 0,
    .kd = 300,
    .max_out = 4 * PI,
    .max_iout = 0.25 * PI,
    .deadband = 0,                                        // 死区
    .A = 0,                                               // 变速积分所能达到的最大值为A+B
    .B = 0,                                               // 启动变速积分的死区
    .output_filtering_coefficient = 0.1,                  // 输出滤波系数
    .derivative_filtering_coefficient = 0,                // 微分滤波系数
    .mode = control::ConstrainedPID::Integral_Limit |     // 积分限幅
            control::ConstrainedPID::OutputFilter |       // 输出滤波
            control::ConstrainedPID::Trapezoid_Intergral  // 梯形积分
};

