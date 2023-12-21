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

#include "bsp_print.h"
#include "can_bridge.h"
#include "cmsis_os.h"
#include "dbus.h"
#include "main.h"

static bsp::CAN* can1 = nullptr;
static remote::DBUS* dbus = nullptr;
static communication::CanBridge* can_bridge = nullptr;

void RM_RTOS_Init() {
    HAL_Delay(200);
    print_use_uart(&huart8);
    can1 = new bsp::CAN(&hcan1, true);
    can_bridge = new communication::CanBridge(can1, 0x51);

    dbus = new remote::DBUS(&huart1);
    HAL_Delay(300);
}

void RM_RTOS_Default_Task(const void* args) {
    UNUSED(args);

    osDelay(500);  // DBUS initialization needs time

    communication::can_bridge_ext_id_t ext_id;
    communication::can_bridge_data_t can_bridge_data;
    while (true) {
        ext_id.data.type = communication::CAN_BRIDGE_TYPE_TWO_FLOAT;
        ext_id.data.rx_id = 0x52;
        if (dbus->swr == remote::DOWN) {
            {
                ext_id.data.reg = 0x71;

                can_bridge_data.data_two_float.data[0] = (float)0;
                can_bridge_data.data_two_float.data[1] = 0.0f;

                can_bridge->Send(ext_id, can_bridge_data);
                osDelay(10);
            }
            continue;
        }
        {
            ext_id.data.reg = 0x70;

            can_bridge_data.data_two_float.data[0] = (float)dbus->ch0;
            can_bridge_data.data_two_float.data[1] = (float)dbus->ch1;

            can_bridge->Send(ext_id, can_bridge_data);
            osDelay(1);
        }
        {
            ext_id.data.reg = 0x71;

            can_bridge_data.data_two_float.data[0] = (float)1;
            can_bridge_data.data_two_float.data[1] = (float)dbus->ch2;

            can_bridge->Send(ext_id, can_bridge_data);
            osDelay(1);
        }
        {
            ext_id.data.reg = 0x72;

            can_bridge_data.data_two_float.data[0] = (float)0;
            can_bridge_data.data_two_float.data[1] = 200.0f;

            can_bridge->Send(ext_id, can_bridge_data);
            osDelay(1);
        }
        {
            ext_id.data.reg = 0x73;

            can_bridge_data.data_two_float.data[0] = 120.0f;
            can_bridge_data.data_two_float.data[1] = 60.0f;

            can_bridge->Send(ext_id, can_bridge_data);
            osDelay(1);
        }

        osDelay(6);
    }
}
