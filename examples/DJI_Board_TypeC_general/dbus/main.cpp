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
#include "dbus.h"

static remote::DBUS* dbus;

void RM_RTOS_Init(void) {
    print_use_uart(&huart1);
    dbus = new remote::DBUS(&huart3);
}

void RM_RTOS_Default_Task(const void* arguments) {
    UNUSED(arguments);

    // NOTE(alvin): print is split because of stack usage is almost reaching
    // limits
    while (true) {
        set_cursor(0, 0);
        clear_screen();
        print(
            "CH0: %-4d CH1: %-4d CH2: %-4d CH3: %-4d \r\nSWL: %d SWR: %d "
            "TWL: %d "
            "@ %d "
            "ms\r\n",
            dbus->ch0, dbus->ch1, dbus->ch2, dbus->ch3, dbus->swl, dbus->swr, dbus->ch4,
            dbus->timestamp);
        print("W: %d A: %d S: %d D: %d\r\n", dbus->keyboard.bit.W, dbus->keyboard.bit.A,
              dbus->keyboard.bit.S, dbus->keyboard.bit.D);
        print("Q: %d E: %d R: %d F: %d G: %d\r\n", dbus->keyboard.bit.Q, dbus->keyboard.bit.E,
              dbus->keyboard.bit.R, dbus->keyboard.bit.F, dbus->keyboard.bit.G);
        print("Z: %d X: %d C: %d V: %d B: %d\r\n", dbus->keyboard.bit.Z, dbus->keyboard.bit.X,
              dbus->keyboard.bit.C, dbus->keyboard.bit.V, dbus->keyboard.bit.B);
        print("SHIFT: %d CTRL: %d\r\n", dbus->keyboard.bit.SHIFT, dbus->keyboard.bit.CTRL);
        print("Mouse: x: %d y: %d z: %d l: %d r: %d\r\n", dbus->mouse.x, dbus->mouse.y,
              dbus->mouse.z, dbus->mouse.l, dbus->mouse.r);

        osDelay(50);
    }
}
