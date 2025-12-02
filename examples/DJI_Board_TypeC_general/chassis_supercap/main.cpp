// Copyright (c) 2025. BNU-HKBU UIC RoboMaster
//
// This program is free software: you can redistribute it
// and/or modify it under the terms of the GNU General
// Public License as published by the Free Software
// Foundation, either version 3 of the License, or (at
// your option) any later version.
//
// This program is distributed in the hope that it will be
// useful, but WITHOUT ANY WARRANTY; without even
// the implied warranty of MERCHANTABILITY or FITNESS
// FOR A PARTICULAR PURPOSE.  See the GNU General
// Public License for more details.
//
// You should have received a copy of the GNU General
// Public License along with this program.  If not, see
// <https://www.gnu.org/licenses/>.

//
// Created by Peter Jiang on 25-9-18.
//
#include "main.h"

#include "bsp_print.h"
#include "chassis.h"
#include "cmsis_os.h"
#include "dbus.h"

bsp::CAN* can1 = nullptr;
bsp::CAN* can2 = nullptr;

driver::Adernal_SuperCap* adernal_supercap = nullptr;
driver::MotorCANBase* fl_motor = nullptr;
driver::MotorCANBase* fr_motor = nullptr;
driver::MotorCANBase* bl_motor = nullptr;
driver::MotorCANBase* br_motor = nullptr;

control::Chassis* chassis = nullptr;
remote::DBUS* dbus = nullptr;

void RM_RTOS_Init() {
    HAL_Delay(200);
    // uart2 use huart1
    print_use_uart(&huart1); // use print() to send debugging message
    print("Initializing...\r\n");

    // use can1 to control the supercap
    can1 = new bsp::CAN(&hcan1, true);
    adernal_supercap = new driver::Adernal_SuperCap(can1);
    print("CAN1 initialized and Adernal_SuperCap created\r\n");

    // use can2 to control the motor
    can2 = new bsp::CAN(&hcan2, false);
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

    driver::MotorCANBase* motors[control::FourWheel::motor_num];
    motors[control::FourWheel::front_left] = fl_motor;
    motors[control::FourWheel::front_right] = fr_motor;
    motors[control::FourWheel::back_left] = bl_motor;
    motors[control::FourWheel::back_right] = br_motor;
    print("Motors initialized\r\n");

    // init chassis data
    control::chassis_t chassis_data;
    chassis_data.motors = motors;
    chassis_data.model = control::CHASSIS_MECANUM_WHEEL;
    chassis_data.has_super_capacitor = true;
    chassis_data.super_capacitor = adernal_supercap;
    chassis = new control::Chassis(chassis_data);

    // enable remote
    dbus = new remote::DBUS(&huart3);
    HAL_Delay(300);

    print("Initializing done!\r\n");
}

void RM_RTOS_Default_Task(const void* args) {
    UNUSED(args);
    print("Waiting for SuperCap to be ready...\r\n");
    osDelay(3000);
    print("Try to config the SuperCap:\r\n");
    // 初始化超级电容 - 选择类型1 (最大24V)
    if (adernal_supercap->initialize(driver::Adernal_Init_24V)) {
        print("SuperCap initialized with Type 1 (24V)\r\n");
    } else {
        print("Failed to initialize SuperCap\r\n");
    }

    osDelay(100);

    // 维持电源输出为60W，Work模式，不开启Exceed
    if (adernal_supercap->setControl(60, driver::Adernal_CtrlMode_Work,
                                     driver::Adernal_CtrlExceed_Off)) {
        print("Set Expect Power to 60W, Work mode, Exceed OFF\r\n");
                                     } else {
                                         print("Failed to set SuperCap parameters\r\n");
                                     }

    while (true) {
        const float ratio = 3.0f / 660.0f * 6 * PI; // 直接修改这个来改转速
        // print("%d %d %d %d", dbus->ch0, dbus->ch1, dbus->ch2);
        // encounter any remote bug when debugging, try to reset the MCU
        chassis->SetSpeed(dbus->ch0 * ratio, dbus->ch1 * ratio, dbus->ch2 * ratio);

        // Kill switch
        if (dbus->swl == remote::UP || dbus->swl == remote::DOWN) {
            RM_ASSERT_TRUE(false, "Operation killed");
        }
        // use new power limit method
        // chassis->UpdatePower(true, 200, 24, 80);
        chassis->Update();

        // 处理超级电容就绪状态打印
        // 处理就绪状态打印
        if (adernal_supercap->hasNewReadyStatus()) {
            print("SuperCap Ready: %s\r\n", adernal_supercap->isReady() ? "Yes" : "No");
            adernal_supercap->clearReadyFlag();
        }

        // 处理反馈数据打印
        if (adernal_supercap->hasNewFeedback()) {
            const driver::Adernal_Fb_Typedef& feedback = adernal_supercap->getFeedback();
            print("Voltage: %.2fV Power: %.2fW Work1:%d%% Work2:%d%%\r\n", feedback.Voltage_NoESR,
                  feedback.Power_Battery, feedback.Work_Sentry1, feedback.Work_Sentry2);
            adernal_supercap->clearFeedbackFlag();
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