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
static bsp::CAN *can1 = nullptr;
static bsp::CAN *can2 = nullptr;

static driver::Motor2006 *pitch_motor_l = nullptr;
static driver::Motor2006 *pitch_motor_r = nullptr;
static driver::Motor2006 *yaw_motor = nullptr;

static driver::Motor3508 *launch_motor_l = nullptr;
static driver::Motor3508 *launch_motor_r = nullptr;

static remote::SBUS* sbus = nullptr;

void RM_RTOS_Init() {
    print_use_uart(&BOARD_UART1);
    can1 = new bsp::CAN(&hcan1);
    can2 = new bsp::CAN(&hcan2);

    static const control::ConstrainedPID::PID_Init_t pitch_motor_omega_pid_init = {
        .kp = 1000,
        .ki = 0,
        .kd = 0,
        .max_out = 10000,
        .max_iout = 4000,
        .deadband = 0,                          // 死区
        .A = 3 * PI,                            // 变速积分所能达到的最大值为A+B
        .B = 2 * PI,                            // 启动变速积分的死区
        .output_filtering_coefficient = 0.1,    // 输出滤波系数
        .derivative_filtering_coefficient = 0,  // 微分滤波系数
        .mode = control::ConstrainedPID::Integral_Limit |        // 积分限幅
                control::ConstrainedPID::OutputFilter |          // 输出滤波
                control::ConstrainedPID::Trapezoid_Intergral |   // 梯形积分
                control::ConstrainedPID::ChangingIntegralRate |  // 变速积分
                control::ConstrainedPID::ErrorHandle,            // 错误处理

    };
    control::ConstrainedPID::PID_Init_t pitch_motor_theta_pid_init = {
        .kp = 8,
        .ki = 0,
        .kd = 200,
        .max_out = 8 * PI,
        .max_iout = PI / 8,
        .deadband = 0,
        .A = 0,                                    // 变速积分所能达到的最大值为A+B
        .B = 0,                                    // 启动变速积分的死区
        .output_filtering_coefficient = 0.5,       // 输出滤波系数
        .derivative_filtering_coefficient = 0.05,  // 微分滤波系数
        .mode = control::ConstrainedPID::OutputFilter | control::ConstrainedPID::DerivativeFilter |
                control::ConstrainedPID::Integral_Limit};

    pitch_motor_l = new driver::Motor2006(can1, 0x201);
    pitch_motor_r = new driver::Motor2006(can1, 0x202);
    yaw_motor = new driver::Motor2006(can1, 0x203);

    pitch_motor_l->SetTransmissionRatio(36);
    pitch_motor_r->SetTransmissionRatio(36);
    yaw_motor->SetTransmissionRatio(36);

    pitch_motor_l->ReInitPID(pitch_motor_omega_pid_init, driver::MotorCANBase::OMEGA);
    pitch_motor_r->ReInitPID(pitch_motor_omega_pid_init, driver::MotorCANBase::OMEGA);
    yaw_motor->ReInitPID(pitch_motor_omega_pid_init, driver::MotorCANBase::OMEGA);
    pitch_motor_l->ReInitPID(pitch_motor_theta_pid_init, driver::MotorCANBase::THETA);
    pitch_motor_r->ReInitPID(pitch_motor_theta_pid_init, driver::MotorCANBase::THETA);
    yaw_motor->ReInitPID(pitch_motor_theta_pid_init, driver::MotorCANBase::THETA);

    pitch_motor_l->SetMode(driver::MotorCANBase::OMEGA | driver::MotorCANBase::THETA);
    pitch_motor_r->SetMode(driver::MotorCANBase::OMEGA | driver::MotorCANBase::THETA);
    yaw_motor->SetMode(driver::MotorCANBase::OMEGA | driver::MotorCANBase::THETA);

    control::ConstrainedPID::PID_Init_t launch_motor_omega_pid_init = {
        .kp = 2500,
        .ki = 3,
        .kd = 0,
        .max_out = 30000,
        .max_iout = 10000,
        .deadband = 0,                          // 死区
        .A = 3 * PI,                            // 变速积分所能达到的最大值为A+B
        .B = 2 * PI,                            // 启动变速积分的死区
        .output_filtering_coefficient = 0.1,    // 输出滤波系数
        .derivative_filtering_coefficient = 0,  // 微分滤波系数
        .mode = control::ConstrainedPID::Integral_Limit |       // 积分限幅
                control::ConstrainedPID::OutputFilter |         // 输出滤波
                control::ConstrainedPID::Trapezoid_Intergral |  // 梯形积分
                control::ConstrainedPID::ChangingIntegralRate,  // 变速积分
    };
    control::ConstrainedPID::PID_Init_t launch_motor_theta_pid_init = {
        .kp = 8,
        .ki = 0,
        .kd = 200,
        .max_out = 8 * PI,
        .max_iout = PI / 8,
        .deadband = 0,
        .A = 0,                                    // 变速积分所能达到的最大值为A+B
        .B = 0,                                    // 启动变速积分的死区
        .output_filtering_coefficient = 0.5,       // 输出滤波系数
        .derivative_filtering_coefficient = 0.05,  // 微分滤波系数
        .mode = control::ConstrainedPID::OutputFilter | control::ConstrainedPID::DerivativeFilter |
                control::ConstrainedPID::Integral_Limit};

    launch_motor_l = new driver::Motor3508(can2, 0x205);
    launch_motor_r = new driver::Motor3508(can2, 0x206);

    launch_motor_l->SetTransmissionRatio(19);
    launch_motor_r->SetTransmissionRatio(19);

    launch_motor_l->ReInitPID(launch_motor_omega_pid_init, driver::MotorCANBase::OMEGA);
    launch_motor_r->ReInitPID(launch_motor_omega_pid_init, driver::MotorCANBase::OMEGA);
    launch_motor_l->ReInitPID(launch_motor_theta_pid_init, driver::MotorCANBase::THETA);
    launch_motor_r->ReInitPID(launch_motor_theta_pid_init, driver::MotorCANBase::THETA);

    launch_motor_l->SetMode(driver::MotorCANBase::OMEGA | driver::MotorCANBase::THETA);
    launch_motor_r->SetMode(driver::MotorCANBase::OMEGA | driver::MotorCANBase::THETA);

    sbus = new remote::SBUS(&BOARD_DBUS);

    osDelay(1000);
}

bool check_enabled()
{
    if (!sbus->IsOnline())
    {
        pitch_motor_l->Disable();
        pitch_motor_r->Disable();
        yaw_motor->Disable();
        launch_motor_l->Disable();
        launch_motor_r->Disable();
        osDelay(10);
        return false;
    }
    else
    {
        pitch_motor_l->Enable();
        pitch_motor_r->Enable();
        yaw_motor->Enable();
        launch_motor_l->Enable();
        launch_motor_r->Enable();
        return true;
    }
}

void RM_RTOS_Default_Task(const void* args) {
    UNUSED(args);

    float pitch_angle = 0;
    float yaw_angle = 0;
    float launch_angle = 0;

    while (true) {
        if (!check_enabled())
        {
            osDelay(10);
            continue;
        }

        const float pitch_ratio = 1.0 / sbus->ROCKER_MAX * 2 * PI;

        pitch_angle += sbus->ch1 * pitch_ratio;
        yaw_angle += sbus->ch2 * pitch_ratio;
        launch_angle += sbus->ch3 * pitch_ratio;

        pitch_motor_l->SetTarget(pitch_angle, true);
        pitch_motor_r->SetTarget(pitch_angle, true);
        yaw_motor->SetTarget(yaw_angle, true);
        launch_motor_l->SetTarget(launch_angle, true);
        launch_motor_r->SetTarget(launch_angle, true);

        osDelay(10);
    }
}
