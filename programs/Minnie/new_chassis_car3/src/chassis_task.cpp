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

#include "chassis.h"
#include "chassis_task.h"
#include "bsp_os.h"
#include "bsp_print.h"
#include "user_define.h"

driver::MotorCANBase* fl_motor = nullptr;
driver::MotorCANBase* fr_motor = nullptr;
driver::MotorCANBase* bl_motor = nullptr;
driver::MotorCANBase* br_motor = nullptr;
control::Chassis* chassis = nullptr;
bsp::CAN* can1 = nullptr;
bsp::CAN* can2 = nullptr;

driver::SuperCap* super_cap = nullptr;
communication::CanBridge* can_bridge = nullptr;

float ch0 = 0;
float ch1 = 0;
float ch2 = 0;
float ch3 = 0;

void update_channel_data(communication::can_bridge_ext_id_t ext_id,
                         communication::can_bridge_data_t data, void* args) {
    UNUSED(args);
    if (ext_id.data.tx_id == 0x51) {
        if (ext_id.data.type == communication::CAN_BRIDGE_TYPE_FOUR_INT16) {
            ch0 = data.data_four_int16.data[0];
            ch1 = data.data_four_int16.data[1];
            ch2 = data.data_four_int16.data[2];
            ch3 = data.data_four_int16.data[3];

        }
    }
}

void init_chassis() {

    // 初始化CAN
    can1 = new bsp::CAN(&hcan1, true, 8);

    can2 = new bsp::CAN(&hcan2, false);

    // 初始化电机
    fl_motor = new driver::Motor3508(can2, 0x203);
    fr_motor = new driver::Motor3508(can2, 0x204);
    bl_motor = new driver::Motor3508(can2, 0x202);
    br_motor = new driver::Motor3508(can2, 0x201);
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
    fl_motor->SetTransmissionRatio(19);

    fr_motor->ReInitPID(omega_pid_init, driver::MotorCANBase::OMEGA);
    fr_motor->SetMode(driver::MotorCANBase::OMEGA);
    fr_motor->SetTransmissionRatio(19);

    bl_motor->ReInitPID(omega_pid_init, driver::MotorCANBase::OMEGA);
    bl_motor->SetMode(driver::MotorCANBase::OMEGA);
    bl_motor->SetTransmissionRatio(19);

    br_motor->ReInitPID(omega_pid_init, driver::MotorCANBase::OMEGA);
    br_motor->SetMode(driver::MotorCANBase::OMEGA);
    br_motor->SetTransmissionRatio(19);

    driver::MotorCANBase* motors[control::FourWheel::motor_num];
    motors[control::FourWheel::front_left] = fl_motor;
    motors[control::FourWheel::front_right] = fr_motor;
    motors[control::FourWheel::back_left] = bl_motor;
    motors[control::FourWheel::back_right] = br_motor;

    // driver::supercap_init_t supercap_init = {
    //     .can = can2,
    //     .tx_id = 0x02e,
    //     .tx_settings_id = 0x02f,
    //     .rx_id = 0x030,
    // };
    // super_cap = new driver::SuperCap(supercap_init);
    // super_cap->Disable();
    // super_cap->TransmitSettings();
    // super_cap->Enable();
    // super_cap->TransmitSettings();
    // super_cap->SetMaxVoltage(24.0f);
    // super_cap->SetPowerTotal(100.0f);
    // super_cap->SetMaxChargePower(150.0f);
    // super_cap->SetMaxDischargePower(250.0f);
    // super_cap->SetPerferBuffer(50.0f);

    can_bridge = new communication::CanBridge(can1, 0x52);


    // 底盘初始化
    control::chassis_t chassis_data;
    chassis_data.motors = motors;
    chassis_data.model = control::CHASSIS_OMNI_WHEEL;
    chassis_data.omni_wheel_setup_.Center_point_angle = 45;
    chassis_data.omni_wheel_setup_.chassis_radius = 23;

    // chassis_data.has_super_capacitor = false;  // 是否有超级电容
    // chassis_data.super_capacitor = super_cap;
    chassis = new control::Chassis(chassis_data);

    // chassis->SetMaxMotorSpeed(2 * PI * 7);

    chassis->CanBridgeSetTxId(0x51);
    can_bridge->RegisterRxCallback(0x70, chassis->CanBridgeUpdateEventXYWrapper, chassis);
    can_bridge->RegisterRxCallback(0x71, chassis->CanBridgeUpdateEventTurnWrapper, chassis);
    can_bridge->RegisterRxCallback(0x72, chassis->CanBridgeUpdateEventPowerLimitWrapper, chassis);
    can_bridge->RegisterRxCallback(0x73, chassis->CanBridgeUpdateEventCurrentPowerWrapper, chassis);
    chassisDebug();
    #ifdef CHASSIS_DEBUG
        can_bridge->RegisterRxCallback(0x74, update_channel_data, nullptr);
        print("DEBUG_ING\r\n");
    #endif

    HAL_Delay(300);
}

void chassisMain() {
    #ifdef CHASSIS_DEBUG
        print("CH0:%d CH1:%d CH2:%d CH3:%d\r\n", ch0, ch1, ch2, ch3);
        chassis->Chassis_DeBug_model(true);
    #endif

    chassis->Update();
    osDelay(10);
}

void chassisDebug() {
    #ifdef CHASSIS_DEBUG
        print("RM_CHASSIS_DEBUG_RING\r\n");
    #else
        print("RM_CHASSIS_RING\r\n");
    #endif

}

void kill_chassis() {
    fl_motor->Disable();
    fr_motor->Disable();
    bl_motor->Disable();
    br_motor->Disable();
}