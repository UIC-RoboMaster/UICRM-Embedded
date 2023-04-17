#pragma once
#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstring>

#include "bsp_batteryvol.h"
#include "chassis_task.h"
#include "cmsis_os2.h"
#include "gimbal_task.h"
#include "imu_task.h"
#include "main.h"
#include "protocol.h"
#include "public_port.h"
#include "referee_task.h"
#include "remote_task.h"
#include "user_define.h"
#include "user_interface.h"
#include "utils.h"
extern osThreadId_t uiTaskHandle;
const osThreadAttr_t uiTaskAttribute = {.name = "uiTask",
                                        .attr_bits = osThreadDetached,
                                        .cb_mem = nullptr,
                                        .cb_size = 0,
                                        .stack_mem = nullptr,
                                        .stack_size = 1024 * 4,
                                        .priority = (osPriority_t)osPriorityBelowNormal,
                                        .tz_module = 0,
                                        .reserved = 0};
void uiTask(void* arg);
void init_ui();
