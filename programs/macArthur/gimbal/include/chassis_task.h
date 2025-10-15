/*###########################################################
 # Copyright (c) 2023-2024. BNU-HKBU UIC RoboMaster         #
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
#include "MotorCanBase.h"
#include "bsp_can.h"
#include "can_bridge.h"
#include "chassis.h"
#include "cmsis_os2.h"
#include "gimbal_task.h"
#include "imu_task.h"
#include "main.h"
#include "public_port.h"
#include "referee_task.h"
#include "remote_task.h"
#include "utils.h"
extern osThreadId_t chassisTaskHandle;
const osThreadAttr_t chassisTaskAttribute = {.name = "chassisTask",
                                             .attr_bits = osThreadDetached,
                                             .cb_mem = nullptr,
                                             .cb_size = 0,
                                             .stack_mem = nullptr,
                                             .stack_size = 512 * 4,
                                             .priority = (osPriority_t)osPriorityAboveNormal,
                                             .tz_module = 0,
                                             .reserved = 0};
void chassisTask(void* arg);
void init_chassis();
void kill_chassis();

extern control::ChassisCanBridgeSender* chassis;

extern float chassis_vx;
extern float chassis_vy;
extern float chassis_vt;
extern bool chassis_boost_flag;

extern const float chassis_max_xy_speed;
extern const float chassis_max_t_speed;