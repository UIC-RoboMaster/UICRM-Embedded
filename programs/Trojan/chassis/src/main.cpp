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
#include "buzzer_task.h"
#include "buzzer_notes.h"

#include <cmath>
#include "DjiMotorBase.h"
#include "bsp_batteryvol.h"
#include "bsp_os.h"
#include "bsp_print.h"
#include "chassis.h"
#include "cmsis_os.h"
#include "user_define.h"

bsp::CAN* can1 = nullptr;
bsp::CAN* can2 = nullptr;
driver::DjiMotorBase* fl_motor = nullptr;
driver::DjiMotorBase* fr_motor = nullptr;
driver::DjiMotorBase* bl_motor = nullptr;
driver::DjiMotorBase* br_motor = nullptr;

control::Chassis* chassis = nullptr;
communication::CanBridge* can_bridge = nullptr;
bsp::BatteryVol* battery_vol = nullptr;

void RM_RTOS_Init() {
    HAL_Delay(100);
    print_use_rtt();

    bsp::SetHighresClockTimer(&BOARD_TIM_SYS);
    init_buzzer();

    can1 = new bsp::CAN(&hcan1);
    can2 = new bsp::CAN(&hcan2);

    fl_motor = new driver::Motor3508(can2, chassis_motor_rx_id[control::FourWheel::front_left]);
    fr_motor = new driver::Motor3508(can2, chassis_motor_rx_id[control::FourWheel::front_right]);
    bl_motor = new driver::Motor3508(can2, chassis_motor_rx_id[control::FourWheel::back_left]);
    br_motor = new driver::Motor3508(can2, chassis_motor_rx_id[control::FourWheel::back_right]);
    driver::DjiMotorBase* motors[] = {fl_motor, fr_motor, bl_motor, br_motor};

    // 底盘标定前的参数检查
    bool configured = CHASSIS_CALIBRATED
                    && std::isfinite(chassis_omega_kp) && chassis_omega_kp > 0
                    && std::isfinite(chassis_omega_ki) && chassis_omega_ki >= 0
                    && std::isfinite(chassis_max_out) && chassis_max_out > 0 && chassis_max_out <= 16384
                    && std::isfinite(chassis_max_motor_speed) && chassis_max_motor_speed > 0;


    control::ConstrainedPID::PID_Init_t omega_pid_init = {
        .kp = chassis_omega_kp,
        .ki = chassis_omega_ki,
        .kd = chassis_omega_kd,
        .max_out = chassis_max_out,
        .max_iout = chassis_max_iout,
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


    for (unsigned i = 0; i < control::FourWheel::motor_num; ++i) {
        motors[i]->Disable();
        motors[i]->ReInitPID(omega_pid_init, driver::DjiMotorBase::OMEGA);
        motors[i]->SetMode(
            driver::DjiMotorBase::OMEGA | (chassis_motor_inverted[i] ? driver::DjiMotorBase::INVERTED : 0)
        );
        if (std::isfinite(chassis_motor_transmission_ratio[i]) && chassis_motor_transmission_ratio[i] > 0)
            motors[i]->SetTransmissionRatio(chassis_motor_transmission_ratio[i]);
        else
            configured = false;
    }

    // 底盘信息
    control::chassis_t chassis_data{};
    chassis_data.motors = motors;
    chassis_data.model = control::CHASSIS_OMNI_WHEEL;
    chassis = new control::Chassis(chassis_data);
    chassis->SetMaxMotorSpeed(chassis_max_motor_speed);
    chassis->SetThreshold(CHASSIS_COMMAND_TIMEOUT);
    chassis->Disable();

    can_bridge = new communication::CanBridge(can1, CAN_BRIDGE_CHASSIS_ID);
    chassis->CanBridgeSetTxId(CAN_BRIDGE_GIMBAL_ID);
    // 如果底盘未被标定则不注册运动回调, 功率回调，防止参数检查被忽略
    if (configured) {
        can_bridge->RegisterRxCallback(CAN_BRIDGE_CHASSIS_XY, chassis->CanBridgeUpdateEventXYWrapper, chassis);
        can_bridge->RegisterRxCallback(CAN_BRIDGE_CHASSIS_TURN, chassis->CanBridgeUpdateEventTurnWrapper, chassis);
        can_bridge->RegisterRxCallback(CAN_BRIDGE_CHASSIS_POWER, chassis->CanBridgeUpdateEventPowerLimitWrapper, chassis);
        can_bridge->RegisterRxCallback(CAN_BRIDGE_CHASSIS_CURRENT_POWER, chassis->CanBridgeUpdateEventCurrentPowerWrapper,chassis);
    }
    battery_vol = new bsp::BatteryVol(&hadc3, ADC_CHANNEL_8, 1, ADC_SAMPLETIME_3CYCLES);

    print("Trojan chassis configured=%d\r\n", configured);
}

void RM_RTOS_Default_Task(const void* args) {
    UNUSED(args);
    Buzzer_Sing(Mario);

    while (true) {
        chassis->UpdatePowerVoltage(battery_vol->GetBatteryVol());
        chassis->Update();
        osDelay(CHASSIS_OS_DELAY);
    }
}
