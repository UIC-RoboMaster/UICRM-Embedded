/*###########################################################
# Copyright (c) 2023. BNU-HKBU UIC RoboMaster              #
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

#include "bsp_gpio.h"
#include "bsp_print.h"
#include "cmsis_os.h"
#include "main.h"
#include "motor.h"

#define KEY_GPIO_GROUP KEY_GPIO_Port
#define KEY_GPIO_PIN KEY_Pin

// Refer to typeA datasheet for channel detail
#define TIM_CLOCK_FREQ 1000000
#define MOTOR_OUT_FREQ 500

driver::MotorPWMBase* motor1;

void RM_RTOS_Init() {
    print_use_uart(&huart1);
    motor1 = new driver::MotorPWMBase(&htim2, 1, TIM_CLOCK_FREQ, MOTOR_OUT_FREQ, 0);

    motor1->SetOutput(1000);
    HAL_Delay(3000);
}

void RM_RTOS_Default_Task(const void* args) {
    UNUSED(args);
    bsp::GPIO key(KEY_GPIO_GROUP, KEY_GPIO_PIN);

    int current = 0;
    while (true) {
        if (key.Read() == 0) {
            osDelay(30);
            if (key.Read() == 1)
                continue;
            while (key.Read() == 0) {
                osDelay(30);
            }
            if (current == 1000) {
                current = 750;
                motor1->SetOutput(current);
            } else {
                current = 1000;
                motor1->SetOutput(current);
            }
            osDelay(200);
        }
    }
}
