#pragma once
#include "bsp_can.h"
#include "buzzer.h"
#include "cmsis_os2.h"
#include "gimbal.h"
#include "imu_task.h"
#include "main.h"
#include "motor.h"
#include "public_port.h"
#include "remote_task.h"
#include "shoot_task.h"
#include "user_define.h"
extern osThreadId_t gimbalTaskHandle;
const osThreadAttr_t gimbalTaskAttribute = {.name = "gimbalTask",
                                            .attr_bits = osThreadDetached,
                                            .cb_mem = nullptr,
                                            .cb_size = 0,
                                            .stack_mem = nullptr,
                                            .stack_size = 512 * 4,
                                            .priority = (osPriority_t)osPriorityHigh,
                                            .tz_module = 0,
                                            .reserved = 0};

void gimbalTask(void* arg);
extern control::Gimbal* gimbal;
extern control::MotorCANBase* pitch_motor;
extern control::MotorCANBase* yaw_motor;
extern control::gimbal_data_t* gimbal_param;
void init_gimbal();
void kill_gimbal();