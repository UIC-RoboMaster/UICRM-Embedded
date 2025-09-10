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
#include "bsp_can.h"
#include "bsp_print.h"
#include "buzzer_notes.h"
#include "buzzer_task.h"
#include "chassis.h"
#include "cmsis_os.h"
#include "supercap.h"

bsp::CAN* can1 = nullptr;
bsp::CAN* can2 = nullptr;
driver::MotorCANBase* fl_motor = nullptr;
driver::MotorCANBase* fr_motor = nullptr;
driver::MotorCANBase* bl_motor = nullptr;
driver::MotorCANBase* br_motor = nullptr;

driver::SuperCap* super_cap = nullptr;

control::Chassis* chassis = nullptr;
communication::CanBridge* can_bridge = nullptr;

void RM_RTOS_Init() {
    HAL_Delay(100);
    print_use_uart(&huart5);
    can2 = new bsp::CAN(&hcan2, false);
    can1 = new bsp::CAN(&hcan1, true);
    fl_motor = new driver::Motor3508(can2, 0x202);
    fr_motor = new driver::Motor3508(can2, 0x201);
    bl_motor = new driver::Motor3508(can2, 0x203);
    br_motor = new driver::Motor3508(can2, 0x204);

    control::ConstrainedPID::PID_Init_t omega_pid_init = {
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

    fl_motor->ReInitPID(omega_pid_init, driver::MotorCANBase::ANGLE_LOOP_CONTROL);
    fl_motor->SetMode(driver::MotorCANBase::ANGLE_LOOP_CONTROL);
    fl_motor->SetTransmissionRatio(19);

    fr_motor->ReInitPID(omega_pid_init, driver::MotorCANBase::ANGLE_LOOP_CONTROL);
    fr_motor->SetMode(driver::MotorCANBase::ANGLE_LOOP_CONTROL);
    fr_motor->SetTransmissionRatio(19);

    bl_motor->ReInitPID(omega_pid_init, driver::MotorCANBase::ANGLE_LOOP_CONTROL);
    bl_motor->SetMode(driver::MotorCANBase::ANGLE_LOOP_CONTROL);
    bl_motor->SetTransmissionRatio(19);

    br_motor->ReInitPID(omega_pid_init, driver::MotorCANBase::ANGLE_LOOP_CONTROL);
    br_motor->SetMode(driver::MotorCANBase::ANGLE_LOOP_CONTROL);
    br_motor->SetTransmissionRatio(19);

    driver::supercap_init_t supercap_init = {
        .can = can1,
        .tx_id = 0x02e,
        .tx_settings_id = 0x02f,
        .rx_id = 0x030,
    };
    super_cap = new driver::SuperCap(supercap_init);
    super_cap->Disable();
    super_cap->TransmitSettings();
    super_cap->Enable();
    super_cap->TransmitSettings();
    super_cap->SetMaxVoltage(24.0f);
    super_cap->SetPowerTotal(100.0f);
    super_cap->SetMaxChargePower(150.0f);
    super_cap->SetMaxDischargePower(250.0f);
    super_cap->SetPerferBuffer(50.0f);

    can_bridge = new communication::CanBridge(can1, 0x52);

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

    chassis->SetMaxMotorSpeed(2 * PI * 8);

    chassis->CanBridgeSetTxId(0x51);
    can_bridge->RegisterRxCallback(0x70, chassis->CanBridgeUpdateEventXYWrapper, chassis);
    can_bridge->RegisterRxCallback(0x71, chassis->CanBridgeUpdateEventTurnWrapper, chassis);
    can_bridge->RegisterRxCallback(0x72, chassis->CanBridgeUpdateEventPowerLimitWrapper, chassis);
    can_bridge->RegisterRxCallback(0x73, chassis->CanBridgeUpdateEventCurrentPowerWrapper, chassis);

    HAL_Delay(300);
    init_buzzer();
}

void RM_RTOS_Default_Task(const void* args) {
    UNUSED(args);

    osDelay(500);
    Buzzer_Sing(Mario);

    while (true) {
        chassis->Update();
        osDelay(10);
    }
}
