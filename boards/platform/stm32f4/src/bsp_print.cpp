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

#include "bsp_print.h"

#include "bsp_uart.h"
#include "bsp_usb.h"
#include "main.h"
#include "printf.h"  // third party tiny-printf implemnetations

#define MAX_PRINT_LEN 1024

bsp::UART* print_uart = NULL;
#ifndef NO_USB
static bsp::VirtualUSB* print_usb = NULL;
#endif
static char print_buffer[MAX_PRINT_LEN];

void print_use_uart(UART_HandleTypeDef* huart, bool dma, uint32_t baudrate) {
    if (print_uart)
        delete print_uart;

    print_uart = new bsp::UART(huart);
    if (baudrate != huart->Init.BaudRate)
        print_uart->SetBaudrate(baudrate);
    print_uart->SetupTx(MAX_PRINT_LEN * 2, dma);  // burst transfer size up to 2x max buffer size
#ifndef NO_USB
    print_usb = NULL;
#endif
}

#ifndef NO_USB
void print_use_usb() {
    if (!print_usb)
        print_usb = new bsp::VirtualUSB();

    print_usb->SetupTx(MAX_PRINT_LEN * 2);  // burst transfer size up to 2x max buffer size
    print_uart = NULL;
}
#endif

int32_t print(const char* format, ...) {
#ifdef NDEBUG
    UNUSED(format);
    UNUSED(print_buffer);
    return 0;
#else  // == #ifdef DEBUG
    va_list args;
    int length;

    va_start(args, format);
    length = vsnprintf(print_buffer, MAX_PRINT_LEN, format, args);
    va_end(args);

    if (print_uart)
        return print_uart->Write((uint8_t*)print_buffer, length);
#ifndef NO_USB
    else if (print_usb)
        return print_usb->Write((uint8_t*)print_buffer, length);
#endif
    else
        return 0;
#endif  // #ifdef NDEBUG
}

void set_cursor(int row, int col) {
    print("\033[%d;%dH", row, col);
}

void clear_screen(void) {
    print("\033[2J");
}
