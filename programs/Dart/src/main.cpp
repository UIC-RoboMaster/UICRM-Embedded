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

#include "main.h"

#include "MotorCanBase.h"
#include "bsp_gpio.h"
#include "bsp_print.h"
#include "cmsis_os.h"
#include "pid.h"
#include "referee_task.h"
#include "sbus.h"
#include "utils.h"

#define KEY_GPIO_GROUP KEY_GPIO_Port
#define KEY_GPIO_PIN KEY_Pin

// Refer to typeC datasheet for channel detail
static bsp::CAN* can2 = nullptr;
static driver::Motor3508* flywheel_motor1 = nullptr;
static driver::Motor3508* flywheel_motor2 = nullptr;
static driver::Motor3508* flywheel_motor3 = nullptr;
static driver::Motor3508* flywheel_motor4 = nullptr;

static bsp::CAN* can1 = nullptr;
static driver::Motor3508* pitch_motor = nullptr;
static driver::Motor6020* yaw_motor = nullptr;
static driver::Motor3508* putter_motor = nullptr;

static remote::SBUS* sbus = nullptr;

const float yaw_offset = 0;
const float yaw_max = 0;
float yaw_angle = yaw_offset;

void RM_RTOS_Init() {
    print_use_usb();
    can1 = new bsp::CAN(&hcan1, true);
    flywheel_motor1 = new driver::Motor3508(can1, 0x201);
    flywheel_motor2 = new driver::Motor3508(can1, 0x202);
    flywheel_motor3 = new driver::Motor3508(can1, 0x203);
    flywheel_motor4 = new driver::Motor3508(can1, 0x204);

    flywheel_motor1->SetTransmissionRatio(1);
    flywheel_motor2->SetTransmissionRatio(1);
    flywheel_motor3->SetTransmissionRatio(1);
    flywheel_motor4->SetTransmissionRatio(1);

    control::ConstrainedPID::PID_Init_t omega_pid_init = {
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
    flywheel_motor1->ReInitPID(omega_pid_init, driver::MotorCANBase::OMEGA);
    flywheel_motor2->ReInitPID(omega_pid_init, driver::MotorCANBase::OMEGA);
    flywheel_motor3->ReInitPID(omega_pid_init, driver::MotorCANBase::OMEGA);
    flywheel_motor4->ReInitPID(omega_pid_init, driver::MotorCANBase::OMEGA);
    flywheel_motor1->SetMode(driver::MotorCANBase::OMEGA | driver::MotorCANBase::INVERTED);
    flywheel_motor2->SetMode(driver::MotorCANBase::OMEGA);
    flywheel_motor3->SetMode(driver::MotorCANBase::OMEGA | driver::MotorCANBase::INVERTED);
    flywheel_motor4->SetMode(driver::MotorCANBase::OMEGA);

    can2 = new bsp::CAN(&hcan2, false);
    pitch_motor = new driver::Motor3508(can2, 0x207);
    yaw_motor = new driver::Motor6020(can2, 0x205);
    putter_motor = new driver::Motor3508(can2, 0x206);

    control::ConstrainedPID::PID_Init_t yaw_motor_theta_pid_init = {
        .kp = 20,
        .ki = 0,
        .kd = 0,
        .max_out = 6 * PI,
        .max_iout = 0,
        .deadband = 0,                                 // 死区
        .A = 0,                                        // 变速积分所能达到的最大值为A+B
        .B = 0,                                        // 启动变速积分的死区
        .output_filtering_coefficient = 0.1,           // 输出滤波系数
        .derivative_filtering_coefficient = 0,         // 微分滤波系数
        .mode = control::ConstrainedPID::OutputFilter  // 输出滤波
    };
    yaw_motor->ReInitPID(yaw_motor_theta_pid_init, driver::MotorCANBase::THETA);
    control::ConstrainedPID::PID_Init_t yaw_motor_omega_pid_init = {
        .kp = 200,
        .ki = 1,
        .kd = 0,
        .max_out = 16384,
        .max_iout = 2000,
        .deadband = 0,                                          // 死区
        .A = 1.5 * PI,                                          // 变速积分所能达到的最大值为A+B
        .B = 1 * PI,                                            // 启动变速积分的死区
        .output_filtering_coefficient = 0.1,                    // 输出滤波系数
        .derivative_filtering_coefficient = 0,                  // 微分滤波系数
        .mode = control::ConstrainedPID::Integral_Limit |       // 积分限幅
                control::ConstrainedPID::OutputFilter |         // 输出滤波
                control::ConstrainedPID::Trapezoid_Intergral |  // 梯形积分
                control::ConstrainedPID::ChangingIntegralRate,  // 变速积分
    };
    yaw_motor->ReInitPID(yaw_motor_omega_pid_init, driver::MotorCANBase::OMEGA);
    yaw_motor->SetMode(driver::MotorCANBase::THETA | driver::MotorCANBase::OMEGA |
                       driver::MotorCANBase::ABSOLUTE);

    control::ConstrainedPID::PID_Init_t motor_3508_omega_pid_init = {
        .kp = 2500,
        .ki = 3,
        .kd = 0,
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
    pitch_motor->SetTransmissionRatio(19);
    pitch_motor->ReInitPID(motor_3508_omega_pid_init, driver::MotorCANBase::OMEGA);
    pitch_motor->SetMode(driver::MotorCANBase::OMEGA);

    putter_motor->SetTransmissionRatio(19);
    putter_motor->ReInitPID(motor_3508_omega_pid_init, driver::MotorCANBase::OMEGA);
    putter_motor->SetMode(driver::MotorCANBase::OMEGA);

    sbus = new remote::SBUS(&huart3);

    HAL_Delay(1000);
}

void RM_RTOS_Default_Task(const void* args) {
    UNUSED(args);

    static BoolEdgeDetector flywheel_switch(false);
    bool flywheel_flag = false;
    const float ratio = 1.0f / 660.0f * PI * 2 * 10;  // 限定电机每秒转10圈
    while (true) {
        flywheel_switch.input(sbus->ch8 > 0);
        if (flywheel_switch.posEdge()) {
            if (flywheel_flag) {
                flywheel_motor1->SetTarget(0);
                flywheel_motor2->SetTarget(0);
                flywheel_motor3->SetTarget(0);
                flywheel_motor4->SetTarget(0);
                flywheel_flag = false;
            } else {
                flywheel_motor1->SetTarget(90 * 2 * PI);
                flywheel_motor2->SetTarget(90 * 2 * PI);
                flywheel_motor3->SetTarget(90 * 2 * PI);
                flywheel_motor4->SetTarget(90 * 2 * PI);
                flywheel_flag = true;
            }
        }
        pitch_motor->SetTarget(sbus->ch3 * ratio);
        putter_motor->SetTarget(sbus->ch2 * ratio);
        yaw_angle += (-sbus->ch4 / 18000.0 / 7.0);
        yaw_angle = clip<float>(yaw_angle, -yaw_max + yaw_offset, yaw_max + yaw_offset);
        yaw_motor->SetTarget(yaw_angle);

        osDelay(1);
    }
}
