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

#define SHOOT_OS_DELAY 1
#define CHASSIS_OS_DELAY 10 - 2
#define GIMBAL_OS_DELAY 1
#define REMOTE_OS_DELAY 1
#define DETECT_OS_DELAY 30
#define UI_OS_DELAY 40
#define SHOOT_REFEREE 0
#define ENABLE_UI 1

typedef struct {
    float pitch;
    float roll;
    float yaw;
} INS_Angle_t;