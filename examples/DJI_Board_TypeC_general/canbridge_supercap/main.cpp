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

#include "adernal_supercap.h"

#include "MotorCanBase.h"
#include "bsp_batteryvol.h"
#include "bsp_can.h"
#include "bsp_print.h"
#include "chassis.h"
#include "cmsis_os.h"

bsp::CAN* can1 = nullptr;
bsp::CAN* can2 = nullptr;
driver::MotorCANBase* fl_motor = nullptr;
driver::MotorCANBase* fr_motor = nullptr;
driver::MotorCANBase* bl_motor = nullptr;
driver::MotorCANBase* br_motor = nullptr;

driver::Adernal_SuperCap* adernal_supercap = nullptr;

control::Chassis* chassis = nullptr;
communication::CanBridge* can_bridge = nullptr;

bsp::BatteryVol* battery_vol = nullptr;

void RM_RTOS_Init() {
    HAL_Delay(100);
    print_use_uart(&huart1);
    can2 = new bsp::CAN(&hcan2, false);
    can1 = new bsp::CAN(&hcan1, true);
    fl_motor = new driver::Motor3508(can2, 0x202);
    fr_motor = new driver::Motor3508(can2, 0x201);
    bl_motor = new driver::Motor3508(can2, 0x203);
    br_motor = new driver::Motor3508(can2, 0x204);

    // 初始化超级电容控制器
    adernal_supercap = new driver::Adernal_SuperCap(can1);

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

    fl_motor->ReInitPID(omega_pid_init, driver::MotorCANBase::ANGLE_LOOP_CONTROL);
    fl_motor->SetMode(driver::MotorCANBase::ANGLE_LOOP_CONTROL);
    fl_motor->SetTransmissionRatio(14);

    fr_motor->ReInitPID(omega_pid_init, driver::MotorCANBase::ANGLE_LOOP_CONTROL);
    fr_motor->SetMode(driver::MotorCANBase::ANGLE_LOOP_CONTROL);
    fr_motor->SetTransmissionRatio(14);

    bl_motor->ReInitPID(omega_pid_init, driver::MotorCANBase::ANGLE_LOOP_CONTROL);
    bl_motor->SetMode(driver::MotorCANBase::ANGLE_LOOP_CONTROL);
    bl_motor->SetTransmissionRatio(14);

    br_motor->ReInitPID(omega_pid_init, driver::MotorCANBase::ANGLE_LOOP_CONTROL);
    br_motor->SetMode(driver::MotorCANBase::ANGLE_LOOP_CONTROL);
    br_motor->SetTransmissionRatio(14);

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
    chassis_data.super_capacitor = adernal_supercap;
    chassis = new control::Chassis(chassis_data);

    chassis->SetMaxMotorSpeed(2 * PI * 7);

    chassis->CanBridgeSetTxId(0x51);
    can_bridge->RegisterRxCallback(0x70, chassis->CanBridgeUpdateEventXYWrapper, chassis);
    can_bridge->RegisterRxCallback(0x71, chassis->CanBridgeUpdateEventTurnWrapper, chassis);
    can_bridge->RegisterRxCallback(0x72, chassis->CanBridgeUpdateEventPowerLimitWrapper, chassis);
    can_bridge->RegisterRxCallback(0x73, chassis->CanBridgeUpdateEventCurrentPowerWrapper, chassis);

    battery_vol = new bsp::BatteryVol(&hadc3, ADC_CHANNEL_8, 1, ADC_SAMPLETIME_3CYCLES);

    HAL_Delay(300);
}

void RM_RTOS_Default_Task(const void* args) {
    UNUSED(args);
    print("Waiting for SuperCap to be ready...\r\n");
    osDelay(3000);
    print("Try to config the SuperCap:\r\n");
    // 初始化超级电容
    if (adernal_supercap->initialize(driver::Adernal_Init_24V)) {
        print("SuperCap initialized with Type 1 (24V)\r\n");
    } else {
        print("Failed to initialize SuperCap\r\n");
    }

    osDelay(100);

    // 维持电源输出最大为60W，Work模式，不开启Exceed
    if (adernal_supercap->setControl(60, driver::Adernal_CtrlMode_Work,
                                     driver::Adernal_CtrlExceed_Off)) {
        print("Set Expect Power to 60W, Work mode, Exceed OFF\r\n");
                                     } else {
                                         print("Failed to set SuperCap parameters\r\n");
                                     }

    while (true) {
        chassis->UpdatePowerVoltage(battery_vol->GetBatteryVol());
        chassis->Update();
        // 处理超级电容就绪状态打印
        // 处理就绪状态打印
        if (adernal_supercap->hasNewReadyStatus()) {
            print("SuperCap Ready: %s\r\n", adernal_supercap->isReady() ? "Yes" : "No");
            adernal_supercap->clearReadyFlag();
        }

        // 处理反馈数据打印和模式切换
        if (adernal_supercap->hasNewFeedback()) {
            const driver::Adernal_Fb_Typedef& feedback = adernal_supercap->getFeedback();
            print("Voltage: %.2fV Power: %.2fW Work1:%d%% Work2:%d%%\r\n", feedback.Voltage_NoESR,
                  feedback.Power_Battery, feedback.Work_Sentry1, feedback.Work_Sentry2);
            adernal_supercap->clearFeedbackFlag();
            // 小于10V时攒一波
            if(adernal_supercap->getCurrentMode() == driver::Adernal_CtrlMode_Work && feedback.Voltage_NoESR <= 10) {
                print("Low voltage, switch to charge mode\r\n");
                adernal_supercap->setControl(60, driver::Adernal_CtrlMode_Charge, driver::Adernal_CtrlExceed_Off);
                chassis->SetSpeedRatio(1.0f); // 恢复限制
            }
            // 充到20V切换回工作模式
            if(adernal_supercap->getCurrentMode() == driver::Adernal_CtrlMode_Charge && feedback.Voltage_NoESR >= 20) {
                print("Charge completed, switch to work mode\r\n");
                adernal_supercap->setControl(60, driver::Adernal_CtrlMode_Work, driver::Adernal_CtrlExceed_Off);
                chassis->SetSpeedRatio(1.5f); // 突破限制
            }
        }

        // 处理安全等级打印
        if (adernal_supercap->hasNewSafetyInfo()) {
            const driver::Adernal_SafetyLevel_Typedef* safety_levels =
                adernal_supercap->getSafetyLevels();
            print("Safety Levels: %d %d %d %d %d %d %d %d\r\n", safety_levels[0], safety_levels[1],
                  safety_levels[2], safety_levels[3], safety_levels[4], safety_levels[5],
                  safety_levels[6], safety_levels[7]);
            adernal_supercap->clearSafetyFlag();
        }
        osDelay(10);
    }
}
