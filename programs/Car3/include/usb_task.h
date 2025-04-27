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
#include "buzzer_notes.h"
#include "buzzer_task.h"
#include "cmsis_os2.h"
#include "gimbal.h"
#include "gimbal_data.h"
#include "imu_task.h"
#include "main.h"
#include "public_port.h"
#include "remote_task.h"
// #include "selftest_task.h"
#include "shoot_task.h"
#include "user_define.h"

int16_t crc_checksum(const int8_t* data, const int length);
void insAngleTransHandle(const float* INS_angle, int16_t* pack);

void init_usb();
void kill_usb();
