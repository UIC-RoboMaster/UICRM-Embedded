/*###########################################################
 # Copyright (c) 2023. BNU-HKBU UIC RoboMaster              #
 #                                                          #
 # This program is free software: you can redistribute it   #
 # and/or modify it under the terms of the GNU General      #
 # Public License as published by the Free Software         #
 # Foundation, either version 3 of the License, or (at      #
 # your option) any later version.                          #
 #                                                          #
 # This program is distributed in the hope that it will be  #
 # useful, but WITHOUT ANY WARRANTY; without even           #
 # the implied warranty of MERCHANTABILITY or FITNESS       #
 # FOR A PARTICULAR PURPOSE.  See the GNU General           #
 # Public License for more details.                         #
 #                                                          #
 # You should have received a copy of the GNU General       #
 # Public License along with this program.  If not, see     #
 # <https://www.gnu.org/licenses/>.                         #
 ###########################################################*/

#pragma once
#include "AHRS.h"
#include "bsp_i2c.h"
#include "bsp_pwm.h"
#include "bsp_spi.h"
#include "cmsis_os2.h"
#include "heater.h"
#include "main.h"
#include "mpu6500.h"
#include "wit_protocol.h"
#define RX_SIGNAL (1 << 0)

extern imu::WITUART* witimu;

extern float yaw_offset;
extern bool imu_ok;

extern imu::MPU6500* mpu6500;
extern control::AHRS* ahrs;

extern osThreadId_t imuTaskHandle;
const osThreadAttr_t imuTaskAttribute = {.name = "imuTask",
                                         .attr_bits = osThreadDetached,
                                         .cb_mem = nullptr,
                                         .cb_size = 0,
                                         .stack_mem = nullptr,
                                         .stack_size = 128 * 4,
                                         .priority = (osPriority_t)osPriorityRealtime,
                                         .tz_module = 0,
                                         .reserved = 0};

void imuTask(void* arg);

extern osThreadId_t extimuTaskHandle;
const osThreadAttr_t extimuTaskAttribute = {.name = "extimuTask",
                                            .attr_bits = osThreadDetached,
                                            .cb_mem = nullptr,
                                            .cb_size = 0,
                                            .stack_mem = nullptr,
                                            .stack_size = 128 * 4,
                                            .priority = (osPriority_t)osPriorityRealtime,
                                            .tz_module = 0,
                                            .reserved = 0};

void extimuTask(void* arg);

void init_imu();

typedef void (*imu_task_delay_t)(uint32_t milli);

void reset_yaw();