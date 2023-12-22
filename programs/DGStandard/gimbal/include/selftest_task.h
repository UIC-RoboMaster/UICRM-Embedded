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
#include "buzzer_notes.h"
#include "buzzer_task.h"
#include "chassis_task.h"
#include "cmsis_os2.h"
#include "gimbal_task.h"
#include "imu_task.h"
#include "main.h"
#include "motor.h"
#include "referee_task.h"
#include "remote_task.h"
#include "shoot_task.h"
#include "user_define.h"

extern osThreadId_t selftestTaskHandle;
const osThreadAttr_t selftestTaskAttribute = {.name = "selftestTask",
                                              .attr_bits = osThreadDetached,
                                              .cb_mem = nullptr,
                                              .cb_size = 0,
                                              .stack_mem = nullptr,
                                              .stack_size = 256 * 4,
                                              .priority = (osPriority_t)osPriorityBelowNormal,
                                              .tz_module = 0,
                                              .reserved = 0};

typedef struct {
    bool fl_motor;
    bool fr_motor;
    bool bl_motor;
    bool br_motor;
    bool yaw_motor;
    bool pitch_motor;
    bool steering_motor;
    bool dbus;
    bool referee;
    bool refereerc;
    bool imu_cali;
    bool imu_temp;
    bool gimbal;
    bool chassis;
    bool shoot;
    bool supercap;
    bool can_bus;
} selftest_t;

extern selftest_t selftest;

void init_selftest();
void selftestTask(void* arg);
