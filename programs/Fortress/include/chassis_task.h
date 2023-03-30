#pragma once
#include "main.h"
#include "cmsis_os2.h"
#include "bsp_can.h"
#include "motor.h"
#include "chassis.h"

#include "imu_task.h"
#include "gimbal_task.h"
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