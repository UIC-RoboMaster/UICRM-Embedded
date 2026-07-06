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

static constexpr float VEL_TARGET = 20.0f;

void RM_RTOS_Init() {
    bsp::SetHighresClockTimer(&BOARD_TIM_SYS);
    print_use_uart(&huart1);

    can1 = new bsp::CAN(&hcan1, true);
    motor1 = new driver::DMMotor4310(can1, 0x009, 0x001, driver::DmControlMode::VEL);

    motor1->Enable();
    HAL_Delay(100);
    motor1->SetTarget(0.0f);
    HAL_Delay(1000);
}

void RM_RTOS_Default_Task(const void* args) {
    UNUSED(args);
    bsp::GPIO key(KEY_GPIO_GROUP, KEY_GPIO_PIN);

    bool running = false;

    while (true) {
        set_cursor(0, 0);
        clear_screen();

        if (key.Read() == 0) {
            osDelay(30);
            if (key.Read() == 1)
                continue;
            while (key.Read() == 0) {
                osDelay(30);
            }
            running = !running;
            motor1->SetTarget(running ? VEL_TARGET : 0.0f);
            osDelay(20);
        }

        motor1->PrintData();
        osDelay(20);
    }
}
