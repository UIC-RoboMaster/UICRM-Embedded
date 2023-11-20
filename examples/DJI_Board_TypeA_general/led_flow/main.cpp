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

#include "main.h"

#include "bsp_gpio.h"
#include "cmsis_os.h"
static bsp::GPIO* led_flow[8] = {nullptr, nullptr, nullptr, nullptr,
                                 nullptr, nullptr, nullptr, nullptr};

void RM_RTOS_Init(void) {
    led_flow[0] = new bsp::GPIO(LED_G1_GPIO_Port, LED_G1_Pin);
    led_flow[1] = new bsp::GPIO(LED_G2_GPIO_Port, LED_G2_Pin);
    led_flow[2] = new bsp::GPIO(LED_G3_GPIO_Port, LED_G3_Pin);
    led_flow[3] = new bsp::GPIO(LED_G4_GPIO_Port, LED_G4_Pin);
    led_flow[4] = new bsp::GPIO(LED_G5_GPIO_Port, LED_G5_Pin);
    led_flow[5] = new bsp::GPIO(LED_G6_GPIO_Port, LED_G6_Pin);
    led_flow[6] = new bsp::GPIO(LED_G7_GPIO_Port, LED_G7_Pin);
    led_flow[7] = new bsp::GPIO(LED_G8_GPIO_Port, LED_G8_Pin);

    HAL_Delay(3000);
}

void RM_RTOS_Default_Task(const void* args) {
    UNUSED(args);
    while (true) {
        for (uint8_t i = 0; i < 8; i++) {
            led_flow[i]->Low();
            osDelay(100);
        }
        for (uint8_t i = 0; i < 8; i++) {
            led_flow[i]->High();
            osDelay(100);
        }
    }
}
