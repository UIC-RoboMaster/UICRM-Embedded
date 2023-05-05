#include "bsp_gpio.h"
#include "bsp_print.h"
#include "cmsis_os.h"
#include "main.h"
#include "motor.h"

#define KEY_GPIO_GROUP KEY_GPIO_Port
#define KEY_GPIO_PIN KEY_Pin

// Refer to typeA datasheet for channel detail
static bsp::CAN* can1 = nullptr;
static control::Motor4310* motor1 = nullptr;

void RM_RTOS_Init() {
    print_use_uart(&huart5);
    can1 = new bsp::CAN(&hcan1, 0x01, true);
    motor1 = new control::Motor4310(can1, 0x30, 0x31, control::MIT);

    // Snail need to be run at idle throttle for some
    HAL_Delay(1000);
}

void RM_RTOS_Default_Task(const void* args) {
    UNUSED(args);
    bsp::GPIO key(KEY_GPIO_GROUP, KEY_GPIO_PIN);
    bsp::GPIO power_output(Power_OUT1_EN_GPIO_Port, Power_OUT1_EN_Pin);
    osDelay(1000);
    power_output.High();
    osDelay(5000);
    int current = 0;
    motor1->MotorEnable();
    while (true) {
        set_cursor(0, 0);
        clear_screen();
        if (key.Read() == 0) {
            osDelay(30);
            if (key.Read() == 1)
                continue;
            while (key.Read() == 0) {
                osDelay(30);
            }
            if (current == 0) {
                current = 10000;
                motor1->SetOutput(12.5, 30, 80, 0.5, 5);
            } else {
                current = 0;
                motor1->SetOutput(-12.5, 30, 80, 0.5, 5);
            }
        }
        motor1->PrintData();
        motor1->TransmitOutput();
        osDelay(50);
    }
}
