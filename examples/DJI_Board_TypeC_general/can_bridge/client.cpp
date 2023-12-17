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
#include "main.h"

static bsp::CAN* can1 = nullptr;
static communication::CanBridge* can_bridge = nullptr;
int16_t ch0 = 0;
int16_t ch1 = 0;
int16_t ch2 = 0;
int16_t ch3 = 0;

void update_channel_data(communication::can_bridge_ext_id_t ext_id,
                         communication::can_bridge_data_t data, void* args) {
    UNUSED(args);
    if (ext_id.data.tx_id == 0x02) {
        if (ext_id.data.type == communication::CAN_BRIDGE_TYPE_FOUR_INT16) {
            ch0 = data.data_four_int16.data[0];
            ch1 = data.data_four_int16.data[1];
            ch2 = data.data_four_int16.data[2];
            ch3 = data.data_four_int16.data[3];
        }
    }
}

void RM_RTOS_Init(void) {
    print_use_uart(&huart1);

    can1 = new bsp::CAN(&hcan1, 0x200, true, 8);
    can_bridge = new communication::CanBridge(can1, 0x01);
    can_bridge->RegisterRxCallback(0x42, update_channel_data, nullptr);
}

void RM_RTOS_Threads_Init(void) {
}

void RM_RTOS_Default_Task(const void* argument) {
    UNUSED(argument);

    while (true) {
        set_cursor(0, 0);
        clear_screen();
        print("CH0:%d CH1:%d CH2:%d CH3:%d", ch0, ch1, ch2, ch3);
        osDelay(20);
    }
}