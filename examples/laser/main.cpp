#include "main.h"

#include "bsp_laser.h"
#include "bsp_print.h"
#include "cmsis_os.h"

static bsp::Laser *laser = nullptr;

void RM_RTOS_Init(void) {
  print_use_uart(&huart1);
  laser = new bsp::Laser(&htim3, 3, 1000000);
}

void RM_RTOS_Default_Task(const void *arguments) {
  UNUSED(arguments);

  while (true) {
    set_cursor(0, 0);
    clear_screen();
    laser->SetVal(0);
    print("laser on\r\n");
    osDelay(1000);
    set_cursor(0, 0);
    clear_screen();
    laser->SetVal(127);
    print("laser on\r\n");
    osDelay(1000);
    set_cursor(0, 0);
    clear_screen();
    laser->SetVal(255);
    print("laser on\r\n");
    osDelay(1000);
    set_cursor(0, 0);
    clear_screen();
    laser->SetVal(127);
    print("laser off\r\n");
    osDelay(1000);
  }
}
