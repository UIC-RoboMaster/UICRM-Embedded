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

#include "DMmotor.h"
#include "bsp_gpio.h"
#include "bsp_print.h"
#include "cmsis_os.h"
#include "main.h"

#ifndef PI
#define PI               3.14159265358979f
#endif

#define KEY_GPIO_GROUP KEY_GPIO_Port
#define KEY_GPIO_PIN KEY_Pin

// Refer to typeA datasheet for channel detail
static bsp::CAN* can1 = nullptr;
static driver::DMMotor4310* motor1 = nullptr;

void RM_RTOS_Init() {
    print_use_uart(&huart5);
    can1 = new bsp::CAN(&hcan1, true);
    motor1 = new driver::DMMotor4310(can1, 0x30, 0x31, driver::POS_VEL);

    // Snail need to be run at idle throttle for some
    HAL_Delay(1000);
}

void RM_RTOS_Default_Task(const void* args) {
    UNUSED(args);
    bsp::GPIO key(KEY_GPIO_GROUP, KEY_GPIO_PIN);
    bsp::GPIO power_output(Power_OUT1_EN_GPIO_Port, Power_OUT1_EN_Pin);
    osDelay(1000);
    power_output.High();
    osDelay(5000);
    int current = 0;
    motor1->MotorEnable();
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
            if (current == 0) {
                current = 10000;
                motor1->SetOutput(motor1->GetTheta() + 2 * PI, 30);
            } else {
                current = 0;
                // motor1->SetOutput(-12.5, 30);
            }
        }
        motor1->PrintData();
        motor1->TransmitOutput();
        osDelay(50);
    }
}
