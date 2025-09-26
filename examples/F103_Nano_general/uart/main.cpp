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
#include "cmsis_os.h"
#include "stm32f1xx_hal_uart.h"

extern UART_HandleTypeDef huart1;

void RM_RTOS_Init(void) {
   print_use_uart(&huart1, true);
   HAL_Delay(10);
}

void RM_RTOS_Default_Task(const void* arg) {
   UNUSED(arg);

   const char* test_msg = "Hello from UART test!\r\n";

   while (true) {
       print(test_msg);

       osDelay(100);
   }
}
