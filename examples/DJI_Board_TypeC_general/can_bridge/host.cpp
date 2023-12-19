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

#include "bsp_can.h"
#include "bsp_gpio.h"
#include "bsp_print.h"
#include "bsp_uart.h"
#include "can_bridge.h"
#include "cmsis_os.h"
#include "dbus.h"
#include "main.h"

static bsp::CAN* can1 = nullptr;
static communication::CanBridge* can_bridge = nullptr;

static remote::DBUS* dbus = nullptr;

void RM_RTOS_Init(void) {
    print_use_usb();

    can1 = new bsp::CAN(&hcan1, true, 8);
    can_bridge = new communication::CanBridge(can1, 0x02);

    dbus = new remote::DBUS(&huart3);
}

void RM_RTOS_Default_Task(const void* argument) {
    UNUSED(argument);

    while (true) {
        communication::can_bridge_data_t data;
        data.data_four_int16.data[0] = dbus->ch0;
        data.data_four_int16.data[1] = dbus->ch1;
        data.data_four_int16.data[2] = dbus->ch2;
        data.data_four_int16.data[3] = dbus->ch3;
        communication::can_bridge_ext_id_t ext_id;
        ext_id.data.rx_id = 0x01;
        ext_id.data.type = communication::CAN_BRIDGE_TYPE_FOUR_INT16;
        ext_id.data.reg = 0x42;
        can_bridge->Send(ext_id, data);
        osDelay(1);
    }
}