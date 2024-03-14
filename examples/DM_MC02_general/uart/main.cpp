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
#include "cmsis_os.h"

#define RX_SIGNAL (1 << 0)

static bsp::UART* UART = nullptr;

static uint8_t* uart_data;

static uint32_t uart_len;

void uartReply(void* arg) {
    UNUSED(arg);
    UART->Write<true>(uart_data, uart_len);
}

void RM_RTOS_Init(void) {
    /// because imu occupies uart1, no other UART can be used, so we need to use UART to print
    // print_use_uart(&huart1);

    UART = new bsp::UART(&huart10);

    UART->SetupTx(200, false);
    UART->SetupRx(200);
    UART->SetupRxData(&uart_data, &uart_len);

    UART->RegisterCallback(uartReply, nullptr);
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