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

#include "bsp_print.h"
#include "cmsis_os.h"
#include "sbus.h"

static remote::SBUS* dbus;

void RM_RTOS_Init(void) {
    print_use_uart(&huart1);
    dbus = new remote::SBUS(&huart3);
}

void RM_RTOS_Default_Task(const void* arguments) {
    UNUSED(arguments);

    // NOTE(alvin): print is split because of stack usage is almost reaching
    // limits
    while (true) {
        set_cursor(0, 0);
        clear_screen();
        print(
            "CH1: %-4d CH2: %-4d CH3: %-4d CH4: %-4d \r\n"
            "CH5: %-4d CH6: %-4d CH7: %-4d CH8: %-4d \r\n"
            "CH9: %-4d CH10: %-4d CH11: %-4d CH12: %-4d \r\n"
            "CH13: %-4d CH14: %-4d CH15: %-4d CH16: %-4d \r\n",
            dbus->ch1, dbus->ch2, dbus->ch3, dbus->ch4, dbus->ch5, dbus->ch6, dbus->ch7, dbus->ch8,
            dbus->ch9, dbus->ch10, dbus->ch11, dbus->ch12, dbus->ch13, dbus->ch14, dbus->ch15,
            dbus->ch16);

        osDelay(50);
    }
}
