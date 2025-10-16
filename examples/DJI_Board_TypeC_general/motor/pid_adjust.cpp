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

static bsp::CAN* can = nullptr;
static driver::Motor2006* motor = nullptr;

void PrintTask(void* argument);

void RM_RTOS_Init() {
    print_use_uart(&huart1, true, 921600);
    can = new bsp::CAN(&hcan2, false);
    motor = new driver::Motor2006(can, 0x203);
    motor->SetTransmissionRatio(36);
    control::ConstrainedPID::PID_Init_t steering_theta_pid_init = {
        .kp = 30,
        .ki = 0,
        .kd = 300,
        .max_out = 4 * PI,
        .max_iout = 0.25 * PI,
        .deadband = 0,                          // 死区
        .A = 0,                                 // 变速积分所能达到的最大值为A+B
        .B = 0,                                 // 启动变速积分的死区
        .output_filtering_coefficient = 0.1,    // 输出滤波系数
        .derivative_filtering_coefficient = 0,  // 微分滤波系数
        .mode = control::ConstrainedPID::Integral_Limit |     // 积分限幅
                control::ConstrainedPID::OutputFilter |       // 输出滤波
                control::ConstrainedPID::Trapezoid_Intergral  // 梯形积分
    };
    motor->ReInitPID(steering_theta_pid_init, driver::MotorCANBase::ANGEL_LOOP_CONTROL);
    control::ConstrainedPID::PID_Init_t omega_pid_init = {
        .kp = 800,
        .ki = 0,
        .kd = 5000,
        .max_out = 10000,
        .max_iout = 0,
        .deadband = 0,                          // 死区
        .A = 2 * PI,                            // 变速积分所能达到的最大值为A+B
        .B = 1.5 * PI,                          // 启动变速积分的死区
        .output_filtering_coefficient = 0.1,    // 输出滤波系数
        .derivative_filtering_coefficient = 0,  // 微分滤波系数
        .mode = control::ConstrainedPID::Integral_Limit |     // 积分限幅
                control::ConstrainedPID::OutputFilter |       // 输出滤波
                control::ConstrainedPID::Trapezoid_Intergral  // 梯形积分
    };
    motor->ReInitPID(omega_pid_init, driver::MotorCANBase::SPEED_LOOP_CONTROL);
    motor->SetMode(driver::MotorCANBase::SPEED_LOOP_CONTROL |
                   driver::MotorCANBase::ANGEL_LOOP_CONTROL | driver::MotorCANBase::ABSOLUTE);

    // Snail need to be run at idle throttle for some
    HAL_Delay(1000);

    xTaskCreate(PrintTask, "PrintTask", 1024, nullptr, osPriorityNormal, nullptr);
}

void RM_RTOS_Default_Task(const void* args) {
    UNUSED(args);
    bsp::GPIO key(KEY_GPIO_GROUP, KEY_GPIO_PIN);
    // while (key.Read() == 1)
    //     osDelay(100);

    //    while (true) {
    //        motor->SetTarget(4 * PI, true);
    //        osDelay(1000);
    //        motor->SetTarget(-4 * PI, true);
    //        osDelay(1000);
    //        motor->SetTarget(0, true);
    //        osDelay(1000);
    //    }

    //    while (true)
    //    {
    //        osDelay(1000);
    //    }
    while (true) {
        for (int i = 0; i < 8; i++) {
            motor->SetTarget(i * PI / 2, true);
            osDelay(2000);
        }
    }
}

void PrintTask(void* argument) {
    UNUSED(argument);
    while (1) {
        control::ConstrainedPID::PID_State_t state =
            motor->GetPIDState(driver::MotorCANBase::ANGEL_LOOP_CONTROL);
        uint8_t buffer[sizeof(state) + 2] = {0xAA, 0xBB};
        memcpy(buffer + 2, &state, sizeof(state));
        dump(&state, sizeof(buffer));
        osDelay(2);
    }
}