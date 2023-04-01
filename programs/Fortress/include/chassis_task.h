#pragma once
#include "bsp_can.h"
#include "chassis.h"
#include "cmsis_os2.h"
#include "gimbal_task.h"
#include "imu_task.h"
#include "main.h"
#include "motor.h"
#include "public_port.h"
#include "remote_task.h"
#include "referee_task.h"
extern osThreadId_t chassisTaskHandle;
const osThreadAttr_t chassisTaskAttribute = {.name = "chassisTask",
                                             .attr_bits = osThreadDetached,
                                             .cb_mem = nullptr,
                                             .cb_size = 0,
                                             .stack_mem = nullptr,
                                             .stack_size = 256 * 4,
                                             .priority = (osPriority_t)osPriorityNormal,
                                             .tz_module = 0,
                                             .reserved = 0};
void chassisTask(void* arg);
void init_chassis();
void kill_chassis();
extern control::Chassis* chassis;