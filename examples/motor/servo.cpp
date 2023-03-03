#include "bsp_gpio.h"
#include "bsp_print.h"
#include "cmsis_os.h"
#include "main.h"
#include "motor.h"
#include "dbus.h"
#include "bsp_os.h"

bsp::CAN *can2 = NULL;
control::MotorCANBase *motor1 = NULL;
control::ServoMotor *load_servo = NULL;
remote::DBUS *dbus = nullptr;
float load_step_angle = 2 * PI / 8;

void RM_RTOS_Init() {
  bsp::SetHighresClockTimer(&htim5);
  HAL_Delay(200);
  print_use_uart(&huart6);
  dbus = new remote::DBUS(&huart3);
  can2 = new bsp::CAN(&hcan2, 0x207, false);
  motor1 = new control::Motor2006(can2, 0x207);
  control::servo_t servo_data;

  servo_data.motor = motor1;
  servo_data.max_speed = 2 * PI;
  servo_data.max_acceleration = 8 * PI;
  servo_data.transmission_ratio = M2006P36_RATIO;
  servo_data.omega_pid_param = new float[3]{25, 5, 22};
  servo_data.max_iout = 1000;
  servo_data.max_out = 10000;


  load_servo = new control::ServoMotor(servo_data);
}

void RM_RTOS_Default_Task(const void *args) {
  UNUSED(args);
  control::MotorCANBase *motors[] = {motor1};
  int last_state = remote::MID;
  int state = 0;

  while (true) {
    if (dbus->swl == remote::UP){
      if(last_state == remote::MID)
        last_state = remote::UP;
    }
    else if (dbus->swl == remote::MID){
      if(last_state == remote::UP){
        last_state = remote::MID;
        load_servo->SetTarget(load_servo->GetTarget() + load_step_angle);
        if(state == 0){
          state=1;
        }
        else{
          state=0;
        }
      }
    }
    load_servo->CalcOutput();


    control::MotorCANBase::TransmitOutput(motors, 1);
    osDelay(1);
  }
}
