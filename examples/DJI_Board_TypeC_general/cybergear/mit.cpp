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
#include "cybergear.h"
#include "main.h"

#define KEY_GPIO_GROUP KEY_GPIO_Port
#define KEY_GPIO_PIN KEY_Pin

// Refer to typeA datasheet for channel detail
static bsp::CAN* can1 = nullptr;
static driver::CyberGear* motor1 = nullptr;

void RM_RTOS_Init() {
    print_use_uart(&huart1);
    can1 = new bsp::CAN(&hcan1, true);
    motor1 = new driver::CyberGear(can1, 0x32, 0x33);
    motor1->SetMode(driver::CYBERGEAR_MODE_MIT);

    // Snail need to be run at idle throttle for some
    HAL_Delay(1000);
}

void RM_RTOS_Default_Task(const void* arguments) {
    UNUSED(arguments);
    osDelay(500);
    motor1->MotorEnable();
    bsp::GPIO key(KEY_GPIO_GROUP, KEY_GPIO_PIN);
    uint16_t current = 0;
    float pos = 12.5;
    while (true) {
        set_cursor(0, 0);
        clear_screen();
        if (key.Read() == 0) {
            while (key.Read() == 0) {
                osDelay(1);
            }
            if (current == 0) {
                current = 10000;
                pos = 12.56;
            } else {
                current = 0;
                pos = -12.56;
            }
        }
        motor1->SetOutput(pos, 30, 80, 50, 0.5);
        motor1->PrintData();
        osDelay(30);
    }
}
