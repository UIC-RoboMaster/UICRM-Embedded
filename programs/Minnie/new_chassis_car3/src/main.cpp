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

#include "main.h"

#include "bsp_print.h"
#include "buzzer_notes.h"
#include "buzzer_task.h"
#include "chassis_task.h"

#define CHASSIS_DEBUG

void RM_RTOS_Init(void) {
    // 初始化串口
    print_use_uart(&huart1, true, 115200);

    init_chassis();
    init_buzzer();
    osDelay(50);
}

void RM_RTOS_Threads_Init(void) {
    buzzerTaskHandle = osThreadNew(buzzerTask, nullptr, &buzzerTaskAttribute);
}

void RM_RTOS_Default_Task(const void* args) {
    UNUSED(args);
    osDelay(100);
    Buzzer_Sing(Mario);
    while (true) {
        chassisMain();
    }
}
