#include "main.h"

#include "bsp_can.h"
#include "bsp_gpio.h"
#include "bsp_print.h"
#include "bsp_uart.h"
#include "cmsis_os2.h"
#include "motor.h"
#include "rgb.h"
#include "supercap.h"
#define KEY_GPIO_GROUP KEY_GPIO_Port
#define KEY_GPIO_PIN KEY_Pin
static bsp::CAN* can1 = NULL;
static control::SuperCap* supercap = NULL;
static display::RGB* rgb = NULL;
static control::Motor3508* motor1 = nullptr;
void RM_RTOS_Init() {
    print_use_uart(&huart1);

    can1 = new bsp::CAN(&hcan1, 0x201, true);
    supercap = new control::SuperCap(can1, 0x211, 0x210);
    rgb = new display::RGB(&htim5, 3, 2, 1, 1000000);
    rgb->Display(0xffff0000);
    motor1 = new control::Motor3508(can1, 0x201);

    HAL_Delay(1000);
}

void RM_RTOS_Default_Task(const void* args) {
    UNUSED(args);
    bsp::GPIO key(KEY_GPIO_GROUP, KEY_GPIO_PIN);
    control::MotorCANBase* motors[] = {motor1};
    int current = 0;
    while (true) {
        rgb->Display(0xff00ff00);
        if (supercap->info.supercap_voltage > 15.0f) {
            supercap->SetTargetPower(120.0);
        }

        osDelay(50);
        set_cursor(0, 0);
        clear_screen();
        print("Input Voltage: %f\r\n", supercap->info.input_voltage);
        print("Input Current: %f\r\n", supercap->info.input_current);
        print("Supercap Voltage: %f\r\n", supercap->info.supercap_voltage);
        print("Target Power: %f\r\n", supercap->info.target_power);
        rgb->Display(0xffff0000);
        if (key.Read() == 0) {
            osDelay(30);
            if (key.Read() == 1)
                continue;
            while (key.Read() == 0) {
                osDelay(30);
            }
            if (current == 0) {
                current = 10000;
                motor1->SetOutput(current);
            } else {
                current = 0;
                motor1->SetOutput(current);
            }
        }
        control::MotorCANBase::TransmitOutput(motors, 1);
        osDelay(50);
    }
}