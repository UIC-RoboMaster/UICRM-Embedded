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
#include "bsp_uart.h"
#include "bsp_usb.h"
#include "cmsis_os.h"

#define RX_SIGNAL (1 << 0)

static bsp::VirtualUSB* USB = nullptr;

static uint8_t* usb_data;

static uint32_t usb_len;

void usbReply(void* arg) {
    UNUSED(arg);
    USB->Write<true>(usb_data, usb_len);
}

void RM_RTOS_Init(void) {
    /// because imu occupies uart1, no other UART can be used, so we need to use USB to print
    print_use_uart(&huart1);

    USB = new bsp::VirtualUSB();

    USB->SetupTx(200);
    USB->SetupRx(200);
    USB->SetupRxData(&usb_data, &usb_len);

    USB->RegisterCallback(usbReply, nullptr);
}

void RM_RTOS_Threads_Init(void) {
}

void RM_RTOS_Default_Task(const void* arg) {
    UNUSED(arg);
    while (true) {
        set_cursor(0, 0);
        clear_screen();

        osDelay(50);
    }
}
