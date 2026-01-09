// Copyright (c) 2025. BNU-HKBU UIC RoboMaster
//
// This program is free software: you can redistribute it
// and/or modify it under the terms of the GNU General
// Public License as published by the Free Software
// Foundation, either version 3 of the License, or (at
// your option) any later version.
//
// This program is distributed in the hope that it will be
// useful, but WITHOUT ANY WARRANTY; without even
// the implied warranty of MERCHANTABILITY or FITNESS
// FOR A PARTICULAR PURPOSE.  See the GNU General
// Public License for more details.
//
// You should have received a copy of the GNU General
// Public License along with this program.  If not, see
// <https://www.gnu.org/licenses/>.

//
// Created by Administrator on 2025/3/15.
//

#pragma once

#include "AHRS.h"
#include "BMI088.h"
#include "IST8310.h"
#include "bsp_gpio.h"
#include "bsp_os.h"
#include "bsp_spi.h"
#include "cmsis_os.h"
#include "heater.h"
#include "main.h"

#define RX_SIGNAL (1 << 0)

extern imu::BMI088* bmi088;
extern control::AHRS* ahrs;

// extern osThreadId_t IMUTaskHandle;
// const osThreadAttr_t IMUTaskAttribute = {.name = "IMU_Task",
//                                             .attr_bits = osThreadDetached,
//                                             .cb_mem = nullptr,
//                                             .cb_size = 0,
//                                             .stack_mem = nullptr,
//                                             .stack_size = 512 * 4,
//                                             .priority = (osPriority_t)osPriorityHigh,
//                                             .tz_module = 0,
//                                             .reserved = 0};
void IMU_print();

void IMU_Init();
// void IMU_Task(void *arg);
