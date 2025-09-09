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
const control::gimbal_data_t gimbal_init_data = {.pitch_offset_ = -2.6F,  // 0.9750f 3.8F 1.5
                                                 .yaw_offset_ = -0.5F,    // 1.1819f -0.5
                                                 .pitch_max_ = 0.545F,    // 0.5039f
                                                 .yaw_max_ = PI,
                                                 .yaw_circle_ = true,
                                                 .pitch_inverted = true,
                                                 .yaw_inverted = false};