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
// Created by Peter Jiang on 25-6-10.
//
#include "main.h"
#include "usart.h"
#include "bsp_can.h"
#include "bsp_print.h"
#include "cmsis_os.h"
#include "adernal_supercap.h"

// CAN实例
static bsp::CAN* can1 = nullptr;
// 超级电容控制器实例
static driver::Adernal_SuperCap* adernal_supercap = nullptr;

void RM_RTOS_Init(void) {
    // 设置串口打印
    print_use_uart(&BOARD_UART2, true, 961200);
    print("Initializing... \r\n");

    // 初始化CAN
    can1 = new bsp::CAN(&hcan1, true);

    // 初始化超级电容控制器
    adernal_supercap = new driver::Adernal_SuperCap(can1);
    print("CAN initialized and Adernal_SuperCap created\r\n");
}

void RM_RTOS_Default_Task(const void* arg) {
    UNUSED(arg);
    print("Waiting for SuperCap to be ready...\r\n");

    // 等待超级电容就绪
    while (!adernal_supercap->isReady()) {
        print("Waiting...\r\n");
        osDelay(100);
    }

    // 初始化超级电容 - 选择类型1 (24V)
    if (adernal_supercap->initialize(driver::Adernal_Init_CapType1)) {
        print("SuperCap initialized with Type 1 (24V)\r\n");
    } else {
        print("Failed to initialize SuperCap\r\n");
    }
    osDelay(500);

    // 设置预期功率为50W，Work模式，不开启Exceed
    if (adernal_supercap->setControl(50, driver::Adernal_CtrlMode_Work, driver::Adernal_CtrlExceed_Off)) {
        print("Set Expect Power to 50W, Work mode, Exceed OFF\r\n");
    } else {
        print("Failed to set SuperCap parameters\r\n");
    }

    while(true) {
        // 处理就绪状态打印
        if (adernal_supercap->hasNewReadyStatus()) {
            print("SuperCap Ready: %s\r\n", adernal_supercap->isReady() ? "Yes" : "No");
            adernal_supercap->clearReadyFlag();
        }

        // 处理反馈数据打印
        if (adernal_supercap->hasNewFeedback()) {
            const driver::Adernal_Fb_Typedef& feedback = adernal_supercap->getFeedback();
            print("Voltage: %.2fV Power: %.2fW Work1:%d%% Work2:%d%%\r\n",
                  feedback.Voltage_NoESR,
                  feedback.Power_Battery,
                  feedback.Work_Sentry1,
                  feedback.Work_Sentry2);
            adernal_supercap->clearFeedbackFlag();
        }

        // 处理安全等级打印
        if (adernal_supercap->hasNewSafetyInfo()) {
            const driver::Adernal_SafetyLevel_Typedef* safety_levels = adernal_supercap->getSafetyLevels();
            print("Safety Levels: %d %d %d %d %d %d %d %d\r\n",
                  safety_levels[0], safety_levels[1],
                  safety_levels[2], safety_levels[3],
                  safety_levels[4], safety_levels[5],
                  safety_levels[6], safety_levels[7]);
            adernal_supercap->clearSafetyFlag();
        }
        osDelay(100);
    }
}