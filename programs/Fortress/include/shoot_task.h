#pragma once
#include "main.h"
#include "cmsis_os2.h"
#include "user_define.h"
#include "bsp_gpio.h"
#include "motor.h"
#include "imu_task.h"
#include "remote_task.h"
extern osThreadId_t shootTaskHandle;
const osThreadAttr_t shootTaskAttribute = {.name = "shootTask",
                                           .attr_bits = osThreadDetached,
                                           .cb_mem = nullptr,
                                           .cb_size = 0,
                                           .stack_mem = nullptr,
                                           .stack_size = 256 * 4,
                                           .priority = (osPriority_t)osPriorityNormal,
                                           .tz_module = 0,
                                           .reserved = 0};

extern control::MotorCANBase* steering_motor;
void shootTask(void* arg);
void init_shoot();