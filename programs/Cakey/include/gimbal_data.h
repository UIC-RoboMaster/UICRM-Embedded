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
#include "gimbal.h"
#include "pid.h"

// basic information of gimbal
const control::gimbal_data_t gimbal_init_data = {
    .pitch_offset_ = 7.16f,
    .yaw_offset_ = 4.6f,
    .pitch_max_ = 0.45f,
    .yaw_max_ = PI,
    .yaw_circle_ = true,
    .pitch_inverted = false,
    .yaw_inverted = true,
    .pitch_eposition = 0,
    .yaw_eposition = 0,
};
