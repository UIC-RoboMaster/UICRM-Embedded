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

#include "shoot_task.h"
#include "MotorCanBase.h"
#include "bsp_can.h"
#include "buzzer_notes.h"
#include "buzzer_task.h"
#include "cmsis_os2.h"
#include "gimbal.h"
#include "gimbal_data.h"
#include "imu_task.h"
#include "main.h"
#include "public_port.h"
#include "remote_task.h"
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
extern driver::MotorCANBase* pitch_motor;
extern driver::MotorCANBase* yaw_motor;
extern control::gimbal_data_t* gimbal_param;
extern float pitch_diff, yaw_diff;
extern INS_Angle_t INS_Angle;
void init_gimbal();
void kill_gimbal();
void Gimbal_Righting();

void debug_gimbal_init();
void debug_gimbal(bool newline = false);
void gimbal_TargeRel(float rel_pitch, float rel_yaw, bool buffer_mode = false);

struct Gimbal_Ratio_data{
    float pitch_ratio_ratio;
    float yaw_ratio_ratio;
};

Gimbal_Ratio_data gimbal_remote_mode();