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
#include "gimbal.h"
#include "pid.h"

// basic information of gimbal
const control::gimbal_data_t gimbal_init_data = {
    .pitch_offset_ = 4.72515f,
    .yaw_offset_ = 3.6478f,
    .pitch_max_ = 0.4253f,
    .yaw_max_ = PI,
};
// basic information of gimbal
extern control::gimbal_pid_t gimbalBasicPID;
void init_gimbalBasicData();

// Spin PID of gimbal
extern control::gimbal_pid_t gimbalSpinPID;
void init_gimbalSpinData();