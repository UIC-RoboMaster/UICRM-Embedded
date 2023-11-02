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
#include "cmsis_os2.h"
#include "bsp_can.h"
#include "motor.h"

static bsp::CAN* can1 = nullptr;
static driver::Motor3508* motor1 = nullptr;
void RM_RTOS_Init(void) {
    HAL_Delay(100);
    can1 = new bsp::CAN(&hcan1, 0x201, true);
    motor1 = new driver::Motor3508(can1, 0x201);
}
void RM_RTOS_Default_Task(const void* arg) {
    UNUSED(arg);
    driver::MotorCANBase* motors[] = {motor1};
        for (int i = 0; i < 3; i++) {
            motor1->SetOutput(10000);
            for(int j=0;j<100;j++) {
                driver::MotorCANBase::TransmitOutput(motors, 1);
                osDelay(10);
            }
            motor1->SetOutput(0);
            for(int j=0;j<100;j++) {
                driver::MotorCANBase::TransmitOutput(motors, 1);
                osDelay(10);
            }

        }
}
