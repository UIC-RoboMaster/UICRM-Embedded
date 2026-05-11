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

#include "bsp_print.h"
#include "bsp_gpio.h"
#include "bsp_os.h"
#include "chassis.h"
#include "cmsis_os.h"
#include "dji_dbus.h"
#include "tim.h"
#include "bsp_dwt.h"

#include "differential_swerve_wheel.h"

bsp::CAN* can = nullptr;
driver::MotorCANBase* motor1 = nullptr;
driver::MotorCANBase* motor2 = nullptr;

void RM_RTOS_Init() {
    HAL_Delay(200);
    print_use_uart(&huart1);
    bsp::SetHighresClockTimer(&htim5);
    can = new bsp::CAN(&hcan1, true);
    motor1 = new driver::Motor3508(can, 0x201);
    motor2 = new driver::Motor3508(can, 0x202);

    control::ConstrainedPID::PID_Init_t omega_pid_init = {
        .kp = 10,
        .ki = 0,
        .kd = 1,  // 4.5
        .max_out = 6 * PI,
        .max_iout = 0,
        .deadband = 0,                                 // 死区
        .A = 0,                                        // 变速积分所能达到的最大值为A+B
        .B = 0,                                        // 启动变速积分的死区
        .output_filtering_coefficient = 0.1,           // 输出滤波系数
        .derivative_filtering_coefficient = 0,         // 微分滤波系数
        .mode = control::ConstrainedPID::OutputFilter  // 输出滤波
    };
    control::ConstrainedPID::PID_Init_t theta_pid_init = {
        .kp = 250,
        .ki = 0,
        .kd = 5,
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

    motor1->ReInitPID(omega_pid_init, driver::MotorCANBase::OMEGA);
    motor1->ReInitPID(theta_pid_init, driver::MotorCANBase::THETA);
    motor1->SetMode(driver::MotorCANBase::THETA);
    motor1->SetTransmissionRatio(19);

    motor2->ReInitPID(omega_pid_init, driver::MotorCANBase::OMEGA);
    motor2->ReInitPID(theta_pid_init, driver::MotorCANBase::THETA);
    motor2->SetMode(driver::MotorCANBase::THETA);
    motor2->SetTransmissionRatio(19);

    DWT_Init(168);
    HAL_Delay(500);
}

void RM_RTOS_Default_Task(const void* args) {
    UNUSED(args);
    float yaw_target = 0.0f;
    float driving_speed_target = 0.1f;
    //key mathematical solver
    control::DifferentialSwerveWheelKinemetic solver;
    solver.ResetTarget(0.0, 0.0);

    int i = 0;
    while (true) {
        osDelay(5);
        if (++i % 10000 == 0) {
            i = 0;
            yaw_target += PI / 2.0;
            yaw_target = yaw_target > 2 * PI ? 0.0 : yaw_target;
        }

        const auto target = solver.UpdateTarget(yaw_target, driving_speed_target);

        print("yaw target:%.3f | drive target:%.3f\r\n", yaw_target, driving_speed_target);
        print("input1 target:%.3f | input2 target:%.3f\r\n", target.motor1_angle, target.motor2_angle);

        motor1->SetTarget(target.motor1_angle * 500);
        motor2->SetTarget(target.motor2_angle * 500);
    }
}