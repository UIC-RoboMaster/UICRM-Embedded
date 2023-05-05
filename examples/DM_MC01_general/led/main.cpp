#include "main.h"

#include "bsp_gpio.h"
#include "cmsis_os.h"
static bsp::GPIO* led = nullptr;

void RM_RTOS_Init(void) {
    led = new bsp::GPIO(LED_GPIO_Port, LED_Pin);
    HAL_Delay(3000);
}

void RM_RTOS_Default_Task(const void* args) {
    UNUSED(args);
    while (true) {
        led->High();
        osDelay(1000);
        led->Low();
        osDelay(1000);
    }
}
