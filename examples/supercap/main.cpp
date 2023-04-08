#include "main.h"
#include "cmsis_os2.h"
#include "bsp_can.h"
#include "supercap.h"
#include "bsp_print.h"
#include "bsp_uart.h"
#include "rgb.h"

bsp::CAN* can1 = NULL;
control::SuperCap* supercap = NULL;
display::RGB* rgb = NULL;
void RM_RTOS_Init() {
    print_use_uart(&huart6);

    can1 = new bsp::CAN(&hcan2, 0x211, false);
    supercap = new control::SuperCap(can1, 0x211,0x210);
    rgb = new display::RGB(&htim5, 3, 2, 1,1000000);
    rgb->Display(0xffff0000);
    HAL_Delay(100);
}

void RM_RTOS_Default_Task(const void* args) {
    UNUSED(args);
    while (true) {
        rgb->Display(0xff00ff00);
        supercap->SetTargetPower(50.0);
        osDelay(50);
        set_cursor(0, 0);
        clear_screen();
        print("Input Voltage: %f\r\n", supercap->info.input_voltage);
        print("Input Current: %f\r\n", supercap->info.input_current);
        print("Supercap Voltage: %f\r\n", supercap->info.supercap_voltage);
        print("Target Power: %f\r\n", supercap->info.target_power);
        rgb->Display(0xffff0000);
        osDelay(50);
    }
}