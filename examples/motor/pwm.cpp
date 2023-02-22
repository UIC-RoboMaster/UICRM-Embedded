#include "bsp_gpio.h"
#include "bsp_print.h"
#include "cmsis_os.h"
#include "main.h"
#include "motor.h"

#define KEY_GPIO_GROUP KEY_GPIO_Port
#define KEY_GPIO_PIN KEY_Pin

// Refer to typeA datasheet for channel detail
#define LEFT_MOTOR_PWM_CHANNEL 1
#define RIGHT_MOTOR_PWM_CHANNEL 2
#define TIM_CLOCK_FREQ 1000000
#define MOTOR_OUT_FREQ 500
#define SNAIL_IDLE_THROTTLE 1080

control::MotorPWMBase *motor1;
control::MotorPWMBase *motor2;

void RM_RTOS_Init() {
  print_use_uart(&huart6);
  motor1 =
      new control::MotorPWMBase(&htim1, LEFT_MOTOR_PWM_CHANNEL, TIM_CLOCK_FREQ,
                                MOTOR_OUT_FREQ, SNAIL_IDLE_THROTTLE);
  motor2 =
      new control::MotorPWMBase(&htim1, RIGHT_MOTOR_PWM_CHANNEL, TIM_CLOCK_FREQ,
                                MOTOR_OUT_FREQ, SNAIL_IDLE_THROTTLE);
  motor1->SetOutput(0);
  motor2->SetOutput(0);
  // Snail need to be run at idle throttle for some
  osDelay(3000);
}

void RM_RTOS_Default_Task(const void *args) {
  UNUSED(args);
  bsp::GPIO key(KEY_GPIO_GROUP, KEY_GPIO_PIN);

  int current = 0;
  while (true) {
    if (key.Read() == 0) {
      osDelay(30);
      if (key.Read() == 1)
        continue;
      while (key.Read() == 0) {
        osDelay(30);
      }
      if (current == 0) {
        current = 500;
        motor1->SetOutput(current);
        motor2->SetOutput(current);
      } else {
        current = 0;
        motor1->SetOutput(current);
        motor2->SetOutput(current);
      }
      osDelay(200);
    }
  }
}
