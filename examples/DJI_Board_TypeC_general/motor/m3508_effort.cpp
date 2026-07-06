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

#include "DjiMotorBase.h"
#include "Motor3508.h"
#include "bsp_gpio.h"
#include "bsp_os.h"
#include "bsp_print.h"
#include "cmsis_os.h"
#include "main.h"
#include "pid.h"
#include "tim.h"

#define KEY_GPIO_GROUP KEY_GPIO_Port
#define KEY_GPIO_PIN KEY_Pin

// EFFORT 趋近阶段使用的力矩幅值 [N·m]，方向由角度误差自动决定
static constexpr float TARGET_TORQUE_NM = 2.5f;
static constexpr float STEP_ANGLE = PI / 3.0f;  // 每次按键转 60°

static bsp::CAN* can2 = nullptr;
static driver::Motor3508* motor1 = nullptr;

void RM_RTOS_Init() {
    bsp::SetHighresClockTimer(&BOARD_TIM_SYS);

    print_use_uart(&huart1);
    can2 = new bsp::CAN(&hcan1, false);
    motor1 = new driver::Motor3508(can2, 0x201);
    motor1->SetTransmissionRatio(71);

    // 到位后 THETA|OMEGA PID 保持用的角度环 / 速度环
    control::ConstrainedPID::PID_Init_t theta_pid_init = {
        .kp = 20,
        .ki = 0,
        .kd = 0,
        .max_out = 15 * PI,
        .max_iout = 0,
        .deadband = 0,
        .A = 0,
        .B = 0,
        .output_filtering_coefficient = 0.1,
        .derivative_filtering_coefficient = 0,
        .mode = control::ConstrainedPID::OutputFilter,
    };
    motor1->ReInitPID(theta_pid_init, driver::DjiMotorBase::THETA);

    control::ConstrainedPID::PID_Init_t omega_pid_init = {
        .kp = 2500,
        .ki = 3,
        .kd = 0,
        .max_out = 30000,
        .max_iout = 10000,
        .deadband = 0,
        .A = 3 * PI,
        .B = 2 * PI,
        .output_filtering_coefficient = 0.1,
        .derivative_filtering_coefficient = 0,
        .mode = control::ConstrainedPID::Integral_Limit | control::ConstrainedPID::OutputFilter |
                control::ConstrainedPID::Trapezoid_Intergral | control::ConstrainedPID::ChangingIntegralRate,
    };
    motor1->ReInitPID(omega_pid_init, driver::DjiMotorBase::OMEGA);

    motor1->SetTorque(TARGET_TORQUE_NM);
    motor1->SetMode(driver::DjiMotorBase::THETA | driver::DjiMotorBase::EFFORT);
    motor1->SetTarget(0.0f);

    HAL_Delay(1000);
}

void RM_RTOS_Default_Task(const void* args) {
    UNUSED(args);
    bsp::GPIO key(KEY_GPIO_GROUP, KEY_GPIO_PIN);

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

            // 每按一次，目标角增加 60°，重新进入恒力矩趋近
            motor1->SetTorque(TARGET_TORQUE_NM);
            motor1->SetMode(driver::DjiMotorBase::THETA | driver::DjiMotorBase::EFFORT);
            motor1->SetTarget(motor1->GetTarget() + STEP_ANGLE, true);
            osDelay(20);
        }

        print("EFFORT demo (M3508 ID1)\r\n");
        print("KEY: +60 deg per press\r\n");
        print("target_torque: %.2f Nm  holding: %s\r\n", TARGET_TORQUE_NM,
              motor1->IsHolding() ? "yes" : "no");
        print("shaft_theta: % .4f  target: % .4f\r\n", motor1->GetOutputShaftTheta(),
              motor1->GetTarget());
        print("feedback_torque: % .4f Nm  output_raw: % d\r\n", motor1->GetTorque(),
              motor1->GetOutput());
        motor1->PrintData();
        osDelay(50);
    }
}
