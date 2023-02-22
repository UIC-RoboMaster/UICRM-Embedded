#include "main.h"

#include "bsp_print.h"
#include "cmsis_os.h"
#include "dbus.h"
#include "gimbal.h"

#define NOTCH (2 * PI / 8)
#define SERVO_SPEED (PI)

bsp::CAN *can1 = nullptr;
bsp::CAN *can2 = nullptr;
control::MotorCANBase *pitch_motor = nullptr;
control::MotorCANBase *yaw_motor = nullptr;

control::gimbal_t gimbal_init_data;
control::Gimbal *gimbal = nullptr;
remote::DBUS *dbus = nullptr;
bool status = false;

void RM_RTOS_Init() {
  print_use_uart(&huart8);
  can1 = new bsp::CAN(&hcan1, 0x201, true);
  can2 = new bsp::CAN(&hcan2, 0x201, false);
  pitch_motor = new control::Motor6020(can1, 0x205);
  yaw_motor = new control::Motor6020(can2, 0x206);
  gimbal_init_data.pitch_motor = pitch_motor;
  gimbal_init_data.yaw_motor = yaw_motor;
  gimbal = new control::Gimbal(gimbal_init_data);

  dbus = new remote::DBUS(&huart1);
}

void RM_RTOS_Default_Task(const void *args) {
  UNUSED(args);

  osDelay(500); // DBUS initialization needs time

  control::MotorCANBase *motors_can1[2];
  control::MotorCANBase *motors_can2[2];
  UNUSED(motors_can2);
  motors_can1[0] = pitch_motor;
  motors_can2[0] = yaw_motor;
  control::gimbal_data_t *gimbal_data = gimbal->GetData();

  while (true) {
    float pitch_ratio = dbus->ch3 / 600.0;
    float yaw_ratio = -dbus->ch2 / 600.0;
    if (dbus->swr == remote::UP) {
      gimbal->TargetAbs(pitch_ratio * gimbal_data->pitch_max_,
                        yaw_ratio * gimbal_data->yaw_max_);
    } else if (dbus->swr == remote::MID) {
      gimbal->TargetRel(pitch_ratio / 30, yaw_ratio / 30);
    }

    // Kill switch
    if (dbus->swl == remote::UP || dbus->swl == remote::DOWN) {
      RM_ASSERT_TRUE(false, "Operation killed");
    }

    gimbal->Update();
    control::MotorCANBase::TransmitOutput(motors_can1, 1);
    control::MotorCANBase::TransmitOutput(motors_can2, 1);
    osDelay(10);
  }
}
