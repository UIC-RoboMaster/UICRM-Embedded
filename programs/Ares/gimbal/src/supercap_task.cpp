// Copyright (c) 2026. BNU-HKBU UIC RoboMaster
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
// Created by gttgf on 2026/3/26.
//

#include "public_port.h"
#include "supercap_task.h"
#include "cmsis_os.h"

osThreadId_t capacityTaskHandle;

driver::Adernal_SuperCap* adernal_supercap = nullptr;
driver::Adernal_Fb_Typedef supercap_feedback = {};
driver::Adernal_StdbyState_Typedef supercap_standby;
const driver::Adernal_SafetyLevel_Typedef* supercap_safety_levels;
float supercap_remaining;

void init_capacity() {
    adernal_supercap = new driver::Adernal_SuperCap(can1);

    // print("Initializing SuperCap:\r\n");

    // 发送初始化帧
    adernal_supercap->initialize(driver::Adernal_Init_30V);

    // print("[SUCCESS] SuperCap initialized successfully (30V mode)\r\n");

    // 设置功率限制
    adernal_supercap->setMaxRefereePower(60);
    adernal_supercap->setMaxChassisPower(150);
}

void capacityTask(void* arg) {
    UNUSED(arg);

    while (true) {
        if (adernal_supercap && adernal_supercap->isValid()) {
            if (adernal_supercap->hasNewReadyStatus()) {
                if(adernal_supercap->isReady()) {
                    adernal_supercap->initialize(driver::Adernal_Init_30V);
                    adernal_supercap->EnableSupercap(true);
                }else {
                    adernal_supercap->EnableSupercap(false);
                }
                //print("SuperCap Ready: %s\r\n", adernal_supercap->isReady() ? "Yes" : "No");
                adernal_supercap->clearReadyFlag();
            }
            if (adernal_supercap->hasNewFeedback()) {
                const auto& [Voltage_NoESR, Power_Battery, Work_Sentry1, Work_Sentry2]
                = adernal_supercap->getFeedback();
                supercap_feedback.Voltage_NoESR = Voltage_NoESR;
                supercap_feedback.Power_Battery = Power_Battery;
                supercap_feedback.Work_Sentry1 = Work_Sentry1;
                supercap_feedback.Work_Sentry2 = Work_Sentry2;
                supercap_remaining = (supercap_feedback.Voltage_NoESR*supercap_feedback.Voltage_NoESR - 10 * 10)/(30 * 30 - 10 * 10);

                adernal_supercap->clearFeedbackFlag();
            }
            if (adernal_supercap->hasNewSafetyInfo()) {
                supercap_safety_levels = adernal_supercap->getSafetyLevels();
                /*
                print("Safety Levels: %d %d %d %d %d %d %d %d\r\n", safety_levels[0], safety_levels[1],
                      safety_levels[2], safety_levels[3], safety_levels[4], safety_levels[5],
                      safety_levels[6], safety_levels[7]);
                */
                adernal_supercap->clearSafetyFlag();
            }
        }
        osDelay(100);
    }
}
void silentMode() {
    adernal_supercap->setControl
    (adernal_supercap->getMaxRefereePower(),
    driver::Adernal_CtrlMode_Silent,
    driver::Adernal_CtrlExceed_Off);
    adernal_supercap->EnableSupercap(false);
}
void chargeMode() {
    if(adernal_supercap->getEnableSupercap()) {
        adernal_supercap->setControl
        (adernal_supercap->getMaxRefereePower(),
        driver::Adernal_CtrlMode_Charge,
        driver::Adernal_CtrlExceed_Off);
    }
}
void workMode() {
    if(adernal_supercap->getEnableSupercap()) {
        adernal_supercap->setControl
        (adernal_supercap->getMaxRefereePower(),
        driver::Adernal_CtrlMode_Work,
        driver::Adernal_CtrlExceed_Off);
    }
}