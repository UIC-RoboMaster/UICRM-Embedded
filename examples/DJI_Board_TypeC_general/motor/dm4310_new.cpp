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

#include "DMmotor.h"
#include "bsp_gpio.h"
#include "bsp_os.h"
#include "bsp_print.h"
#include "cmsis_os.h"
#include "main.h"

#define KEY_GPIO_GROUP KEY_GPIO_Port
#define KEY_GPIO_PIN KEY_Pin

static bsp::CAN* can1 = nullptr;
static driver::DMMotor4310* motor1 = nullptr;

void RM_RTOS_Init() {
    bsp::SetHighresClockTimer(&BOARD_TIM_SYS);
    print_use_uart(&huart1);

    can1 = new bsp::CAN(&hcan1, true);
    motor1 = new driver::DMMotor4310(can1, 0x009, 0x001, driver::VEL);

    // DM4310 needs explicit enable command before accepting runtime control frames.
    motor1->MotorEnable();
    HAL_Delay(100);
    motor1->SetOutput(0.0f);
    motor1->TransmitOutput();
    HAL_Delay(1000);
}

void RM_RTOS_Default_Task(const void* args) {
    UNUSED(args);
    bsp::GPIO key(KEY_GPIO_GROUP, KEY_GPIO_PIN);

    while (true) {
        set_cursor(0, 0);
        clear_screen();
        if (key.Read() == 0) {
            osDelay(30);
            if (key.Read() == 1)
                continue;
            while (key.Read() == 0) {
                motor1->SetOutput(20);
                motor1->TransmitOutput();  // 发送CAN控制帧
                osDelay(30);
            }
            motor1->SetOutput(0);
            motor1->TransmitOutput();  // 停止电机
        }
        motor1->PrintData();
        osDelay(20);
    }
}
