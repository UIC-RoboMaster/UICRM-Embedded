#include "bsp_gpio.h"
#include "cmsis_os.h"
#include "main.h"

static bsp::GPIO *gpio_red, *gpio_green;

void RM_RTOS_Init(void) {
  gpio_red = new bsp::GPIO(LED_RED_GPIO_Port, LED_RED_Pin);
  gpio_green = new bsp::GPIO(LED_GREEN_GPIO_Port, LED_GREEN_Pin);
  gpio_red->High();
  gpio_green->Low();
}

void RM_RTOS_Default_Task(const void* args) {
  UNUSED(args);
  while (true) {
    gpio_red->Toggle();
    gpio_green->Toggle();
    osDelay(500);
  }
}
