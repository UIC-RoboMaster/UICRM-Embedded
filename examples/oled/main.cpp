#include "main.h"

#include "bsp_print.h"
#include "bsp_uart.h"
#include "cmsis_os.h"
#include "oled.h"

static display::OLED* OLED = nullptr;

void RM_RTOS_Init(void) {
  print_use_uart(&huart1);
  OLED = new display::OLED(&hi2c2, 0x3C);
}

void RM_RTOS_Default_Task(const void* arg) {
  UNUSED(arg);

  OLED->ShowRMLOGO();
  osDelay(2000);

  OLED->ShowIlliniRMLOGO();
  osDelay(2000);

  while (true) {
    OLED->DrawCat();
    osDelay(1);
  }
}
