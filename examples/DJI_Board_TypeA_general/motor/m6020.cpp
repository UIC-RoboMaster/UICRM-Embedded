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

bsp::CAN* can1 = NULL;
driver::MotorCANBase* motor1 = NULL;
// control::MotorCANBase* motor2 = NULL;

void RM_RTOS_Init() {
    print_use_uart(&huart8);

    can1 = new bsp::CAN(&hcan1, true);
    motor1 = new driver::Motor6020(can1, 0x209);
    // motor2 = new control::Motor6020(can2, 0x206);
}

void RM_RTOS_Default_Task(const void* args) {
    UNUSED(args);
    driver::MotorCANBase* motors[] = {motor1};

    bsp::GPIO key(KEY_GPIO_Port,KEY_Pin);
    while (true) {
        if (key.Read()) {
            motor1->SetOutput(800);
            // motor2->SetOutput(800);
        } else {
            motor1->SetOutput(0);
            // motor2->SetOutput(0);
        }
        driver::MotorCANBase::TransmitOutput(motors, 1);
        set_cursor(0, 0);
        clear_screen();
        motor1->PrintData();
        osDelay(100);
    }
}
