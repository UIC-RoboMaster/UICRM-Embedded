#include "bsp_gpio.h"
#include "bsp_print.h"
#include "cmsis_os.h"
#include "main.h"
#include "motor.h"



bsp::CAN*can2 = NULL;
control::MotorCANBase* motor1 = NULL;
//control::MotorCANBase* motor2 = NULL;

void RM_RTOS_Init() {
  print_use_uart(&huart1);

  can2 = new bsp::CAN(&hcan2, 0x205, false);
  motor1 = new control::Motor6020(can2, 0x205);
  //motor2 = new control::Motor6020(can2, 0x206);
}

void RM_RTOS_Default_Task(const void* args) {
  UNUSED(args);
  control::MotorCANBase* motors[] = {motor1};

  bsp::GPIO key(KEY_GPIO_Port, KEY_Pin);
  while (true) {
    if (key.Read()) {
      motor1->SetOutput(800);
      //motor2->SetOutput(800);
    } else {
      motor1->SetOutput(0);
      //motor2->SetOutput(0);
    }
    control::MotorCANBase::TransmitOutput(motors, 1);
    set_cursor(0, 0);
    clear_screen();
    motor1->PrintData();
    osDelay(100);
  }
}
