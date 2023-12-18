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

#define KEY_GPIO_GROUP GPIOB
#define KEY_GPIO_PIN GPIO_PIN_2

static bsp::CAN* can1 = nullptr;
static driver::MotorCANBase* motor = nullptr;

void RM_RTOS_Init() {
    print_use_uart(&huart6);

    can1 = new bsp::CAN(&hcan1);
    motor = new driver::Motor6623(can1, 0x205);
}

void RM_RTOS_Default_Task(const void* args) {
    UNUSED(args);
    driver::MotorCANBase* motors[] = {motor};

    bsp::GPIO key(KEY_GPIO_GROUP, KEY_GPIO_PIN);
    while (true) {
        motor->PrintData();
        if (key.Read())
            motor->SetOutput(3000);
        else
            motor->SetOutput(0);
        driver::MotorCANBase::TransmitOutput(motors, 1);
        motor->PrintData();
        osDelay(100);
    }
}