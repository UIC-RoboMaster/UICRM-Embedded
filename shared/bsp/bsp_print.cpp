#include "bsp_print.h"

#include "bsp_uart.h"
#include "bsp_usb.h"
#include "main.h"
#include "printf.h"  // third party tiny-printf implemnetations

#define MAX_PRINT_LEN 256

static bsp::UART* print_uart = NULL;
static bsp::VirtualUSB* print_usb = NULL;
static char print_buffer[MAX_PRINT_LEN];

void print_use_uart(UART_HandleTypeDef* huart) {
    if (print_uart)
        delete print_uart;

    print_uart = new bsp::UART(huart);
    print_uart->SetupTx(MAX_PRINT_LEN * 2);  // burst transfer size up to 2x max buffer size
    print_usb = NULL;
}

void print_use_usb() {
    if (!print_usb)
        print_usb = new bsp::VirtualUSB();

    print_usb->SetupTx(MAX_PRINT_LEN * 2);  // burst transfer size up to 2x max buffer size
    print_uart = NULL;
}

int32_t print(const char* format, ...) {
#ifdef NDEBUG
    UNUSED(format);
    UNUSED(print_buffer);
    return 0;
#else   // == #ifdef DEBUG
    va_list args;
    int length;

    va_start(args, format);
    length = vsnprintf(print_buffer, MAX_PRINT_LEN, format, args);
    va_end(args);

    if (print_uart)
        return print_uart->Write((uint8_t*)print_buffer, length);
    else if (print_usb)
        return print_usb->Write((uint8_t*)print_buffer, length);
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
