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
#include "bsp_can.h"
#include "bsp_print.h"
#include "chassis.h"
#include "cmsis_os.h"
#include "main.h"
#include "public_port.h"
#include "supercap.h"
osThreadId_t chassisTaskHandle;

const float chassis_max_xy_speed = 2 * PI * 10;
const float chassis_max_t_speed = 2 * PI * 5;

float chassis_vx = 0;
float chassis_vy = 0;
float chassis_vt = 0;
bool chassis_boost_flag = true;

driver::MotorCANBase* fl_motor = nullptr;
driver::MotorCANBase* fr_motor = nullptr;
driver::MotorCANBase* bl_motor = nullptr;
driver::MotorCANBase* br_motor = nullptr;

driver::SuperCap* super_cap = nullptr;

control::Chassis* chassis = nullptr;

void init_chassis() {
    HAL_Delay(1000);
    // print_use_uart(&huart1);
    // can1 = new bsp::CAN(&hcan1, true);
    fl_motor = new driver::Motor3508(can1, 0x202);
    fr_motor = new driver::Motor3508(can1, 0x201);
    bl_motor = new driver::Motor3508(can1, 0x203);
    br_motor = new driver::Motor3508(can1, 0x204);

    control::ConstrainedPID::PID_Init_t omega_pid_init = {
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

    fl_motor->ReInitPID(omega_pid_init, driver::MotorCANBase::OMEGA);
    fl_motor->SetMode(driver::MotorCANBase::OMEGA);
    fl_motor->SetTransmissionRatio(14);

    fr_motor->ReInitPID(omega_pid_init, driver::MotorCANBase::OMEGA);
    fr_motor->SetMode(driver::MotorCANBase::OMEGA);
    fr_motor->SetTransmissionRatio(14);

    bl_motor->ReInitPID(omega_pid_init, driver::MotorCANBase::OMEGA);
    // bl_motor->SetMode(driver::MotorCANBase::OMEGA | driver::MotorCANBase::INVERTED);
    bl_motor->SetMode(driver::MotorCANBase::OMEGA);
    bl_motor->SetTransmissionRatio(14);

    br_motor->ReInitPID(omega_pid_init, driver::MotorCANBase::OMEGA);
    // br_motor->SetMode(driver::MotorCANBase::OMEGA | driver::MotorCANBase::INVERTED);
    br_motor->SetMode(driver::MotorCANBase::OMEGA);
    br_motor->SetTransmissionRatio(14);

    //    driver::supercap_init_t supercap_init = {
    //        .can = can2,
    //        .tx_id = 0x02e,
    //        .tx_settings_id = 0x02f,
    //        .rx_id = 0x030,
    //    };
    //    super_cap = new driver::SuperCap(supercap_init);
    //    super_cap->Disable();
    //    super_cap->TransmitSettings();
    //    super_cap->Enable();
    //    super_cap->TransmitSettings();
    //    super_cap->SetMaxVoltage(23.5f);
    //    super_cap->SetPowerTotal(120.0f);
    //    super_cap->SetMaxChargePower(150.0f);
    //    super_cap->SetMaxDischargePower(250.0f);
    //    super_cap->SetPerferBuffer(40.0f);

    driver::MotorCANBase* motors[control::FourWheel::motor_num];
    motors[control::FourWheel::front_left] = fl_motor;
    motors[control::FourWheel::front_right] = fr_motor;
    motors[control::FourWheel::back_left] = bl_motor;
    motors[control::FourWheel::back_right] = br_motor;

    control::chassis_t chassis_data;
    chassis_data.motors = motors;
    chassis_data.model = control::CHASSIS_MECANUM_WHEEL;
    chassis_data.has_super_capacitor = true;
    chassis_data.super_capacitor = super_cap;
    chassis = new control::Chassis(chassis_data);

    chassis->SetMaxMotorSpeed(2 * PI * 7);

    HAL_Delay(300);
}

void chassisTask(void* args) {
    UNUSED(args);

    osDelay(500);

    while (true) {
        chassis->Update();
        osDelay(10);
    }
}

void kill_chassis() {
    chassis->Disable();
}
