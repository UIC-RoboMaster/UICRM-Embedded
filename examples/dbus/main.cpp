#include "main.h"

#include "bsp_print.h"
#include "cmsis_os.h"
#include "dbus.h"

static remote::DBUS* dbus;

void RM_RTOS_Init(void) {
    print_use_uart(&huart6);
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
        osDelay(50);
    }
}
