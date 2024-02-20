/*###########################################################
# Copyright (c) 2023-2024. BNU-HKBU UIC RoboMaster         #
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

#include "MotorCanBase.h"
#include "bsp_gpio.h"
#include "bsp_print.h"
#include "cmsis_os.h"
#include "main.h"
#include "pid.h"

#define KEY_GPIO_GROUP KEY_GPIO_Port
#define KEY_GPIO_PIN KEY_Pin

// Refer to typeA datasheet for channel detail
static bsp::CAN* can1 = nullptr;
static driver::Motor3508* motor1 = nullptr;

void RM_RTOS_Init() {
    print_use_uart(&huart1);
    can1 = new bsp::CAN(&hcan1, true);
    motor1 = new driver::Motor3508(can1, 0x201);
    motor1->SetTransmissionRatio(19);
    control::ConstrainedPID::PID_Init_t omega_pid_init = {
        .kp = 2500,
        .ki = 3,
        .kd = 0,
        .max_out = 30000,
        .max_iout = 10000,
        .deadband = 0,                          // 死区
        .A = 6000,                              // 变速积分所能达到的最大值为A+B
        .B = 4000,                              // 启动变速积分的死区
        .output_filtering_coefficient = 0.1,    // 输出滤波系数
        .derivative_filtering_coefficient = 0,  // 微分滤波系数
        .mode = control::ConstrainedPID::Integral_Limit |       // 积分限幅
                control::ConstrainedPID::OutputFilter |         // 输出滤波
                control::ConstrainedPID::Trapezoid_Intergral |  // 梯形积分
                control::ConstrainedPID::ChangingIntegralRate,  // 变速积分
    };
    motor1->ReInitPID(omega_pid_init, driver::MotorCANBase::OMEGA);
    motor1->SetMode(driver::MotorCANBase::OMEGA);

    // Snail need to be run at idle throttle for some
    HAL_Delay(1000);
}

void RM_RTOS_Default_Task(const void* args) {
    UNUSED(args);
    bsp::GPIO key(KEY_GPIO_GROUP, KEY_GPIO_PIN);
    int current = 0;
    while (true) {
        set_cursor(0, 0);
        clear_screen();
        if (key.Read() == 0) {
            osDelay(30);
            if (key.Read() == 1)
                continue;
            while (key.Read() == 0) {
                osDelay(30);
            }
            if (current == 0) {
                current = 10000;
                motor1->SetTarget(15 * PI);
            } else {
                current = 0;
                motor1->SetTarget(0);
            }

            osDelay(20);
        }
        motor1->PrintData();
        osDelay(20);
    }
}
