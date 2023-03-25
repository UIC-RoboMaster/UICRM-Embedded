#include "main.h"

#include "bsp_gpio.h"
#include "bsp_print.h"
#include "cmsis_os.h"
#include "dbus.h"

void RM_RTOS_Init() { print_use_uart(&huart6); }

void RM_RTOS_Default_Task(const void *arg) {
  UNUSED(arg);
  bsp::GPIO key(BUTTON_TRI_GPIO_Port, BUTTON_TRI_Pin);

  while (true) {
    set_cursor(0, 0);
    clear_screen();
    print("key: %d\n", key.Read() == true ? 1 : 0);
    osDelay(10);
  }
}