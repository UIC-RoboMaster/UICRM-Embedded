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

#include "bsp_print.h"
#include "chassis.h"
#include "cmsis_os.h"
#include "main.h"

bsp::CAN* can1 = nullptr;
bsp::CAN* can2 = nullptr;
driver::MotorCANBase* fl_motor = nullptr;
driver::MotorCANBase* fr_motor = nullptr;
driver::MotorCANBase* bl_motor = nullptr;
driver::MotorCANBase* br_motor = nullptr;

control::Chassis* chassis = nullptr;
communication::CanBridge* can_bridge = nullptr;

void RM_RTOS_Init() {
    HAL_Delay(200);
    print_use_uart(&huart8);
    can1 = new bsp::CAN(&hcan1, true);
    can2 = new bsp::CAN(&hcan2, false);
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
        .A = 6000,                              // 变速积分所能达到的最大值为A+B
        .B = 4000,                              // 启动变速积分的死区
        .output_filtering_coefficient = 0.1,    // 输出滤波系数
        .derivative_filtering_coefficient = 0,  // 微分滤波系数
        .mode = control::ConstrainedPID::Integral_Limit |       // 积分限幅
                control::ConstrainedPID::OutputFilter |         // 输出滤波
                control::ConstrainedPID::Trapezoid_Intergral |  // 梯形积分
                control::ConstrainedPID::ChangingIntegralRate,  // 变速积分
    };

    fl_motor->ReInitPID(omega_pid_init, driver::MotorCANBase::SPEED_LOOP_CONTROL);
    fl_motor->SetMode(driver::MotorCANBase::SPEED_LOOP_CONTROL);
    fl_motor->SetTransmissionRatio(19);

    fr_motor->ReInitPID(omega_pid_init, driver::MotorCANBase::SPEED_LOOP_CONTROL);
    fr_motor->SetMode(driver::MotorCANBase::SPEED_LOOP_CONTROL);
    fr_motor->SetTransmissionRatio(19);

    bl_motor->ReInitPID(omega_pid_init, driver::MotorCANBase::SPEED_LOOP_CONTROL);
    bl_motor->SetMode(driver::MotorCANBase::SPEED_LOOP_CONTROL);
    bl_motor->SetTransmissionRatio(19);

    br_motor->ReInitPID(omega_pid_init, driver::MotorCANBase::SPEED_LOOP_CONTROL);
    br_motor->SetMode(driver::MotorCANBase::SPEED_LOOP_CONTROL);
    br_motor->SetTransmissionRatio(19);

    can_bridge = new communication::CanBridge(can2, 0x52);

    driver::MotorCANBase* motors[control::FourWheel::motor_num];
    motors[control::FourWheel::front_left] = fl_motor;
    motors[control::FourWheel::front_right] = fr_motor;
    motors[control::FourWheel::back_left] = bl_motor;
    motors[control::FourWheel::back_right] = br_motor;

    control::chassis_t chassis_data;
    chassis_data.motors = motors;
    chassis_data.model = control::CHASSIS_MECANUM_WHEEL;
    chassis = new control::Chassis(chassis_data);

    chassis->CanBridgeSetTxId(0x51);

    can_bridge->RegisterRxCallback(0x70, chassis->CanBridgeUpdateEventXYWrapper, chassis);
    can_bridge->RegisterRxCallback(0x71, chassis->CanBridgeUpdateEventTurnWrapper, chassis);
    can_bridge->RegisterRxCallback(0x72, chassis->CanBridgeUpdateEventPowerLimitWrapper, chassis);
    can_bridge->RegisterRxCallback(0x73, chassis->CanBridgeUpdateEventCurrentPowerWrapper, chassis);

    HAL_Delay(300);
}

void RM_RTOS_Default_Task(const void* args) {
    UNUSED(args);

    osDelay(500);  // DBUS initialization needs time

    while (true) {
        chassis->Update();
        osDelay(10);
    }
}
