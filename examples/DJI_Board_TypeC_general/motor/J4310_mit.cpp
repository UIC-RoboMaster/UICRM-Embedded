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

#include "DmMotorBase.h"
#include "bsp_gpio.h"
#include "bsp_os.h"
#include "bsp_print.h"
#include "cmsis_os.h"
#include "main.h"

#define KEY_GPIO_GROUP KEY_GPIO_Port
#define KEY_GPIO_PIN KEY_Pin

static bsp::CAN* can1 = nullptr;
static driver::DMMotor4310* motor1 = nullptr;

// MIT 模式下的控制参数
// Kp: 位置刚度，值越大电机"越硬"，对位置偏差响应越强（必须 > 0 才有位置保持能力）
// Kd: 速度阻尼，值越大电机转动阻力越大，可抑制振荡
static constexpr float MIT_KP = 10.0f;
static constexpr float MIT_KD = 1.5f;

void RM_RTOS_Init() {
    bsp::SetHighresClockTimer(&BOARD_TIM_SYS);
    print_use_uart(&huart1);

    can1 = new bsp::CAN(&hcan1, true);
    motor1 = new driver::DMMotor4310(can1, 0x00, 0x01, driver::DmControlMode::MIT);

    // DM4310 需要显式发送使能命令后才能接受运行时控制帧
    motor1->Enable();
    HAL_Delay(100);

    // 上电后使能电机，Kp > 0 让电机保持在当前位置
    // MIT 模式下后台线程自动以 1kHz 发送控制帧
    motor1->SetTarget(0.0f);
    motor1->SetMitParams(0.0f, MIT_KP, MIT_KD, 0.0f);
    // motor1->SetZeroPos();
    HAL_Delay(1000);
}

void RM_RTOS_Default_Task(const void* args) {
    UNUSED(args);
    bsp::GPIO key(KEY_GPIO_GROUP, KEY_GPIO_PIN);

    while (true) {
        set_cursor(0, 0);
        clear_screen();

        // 按键消抖与切换
        if (key.Read() == 0) {
            osDelay(30);
            if (key.Read() == 1)
                continue;
            while (key.Read() == 0) {
                osDelay(30);
            }
            // 相对当前反馈位置增加 1 rad，v_des=0 纯位置控制
            motor1->SetTarget(motor1->GetTheta() + 1.0f);
            motor1->SetMitParams(0.0f, MIT_KP, MIT_KD, 0.0f);
            osDelay(20);
        }

        motor1->PrintData();
        osDelay(20);
    }
}
