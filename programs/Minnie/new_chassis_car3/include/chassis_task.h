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
#include "chassis.h"
#include "cmsis_os2.h"
#include "main.h"
#include "utils.h"
#include "user_define.h"

typedef enum {
    REMOTE_MODE_FOLLOW = 1,
    REMOTE_MODE_SPIN = 2,
    REMOTE_MODE_ADVANCED = 3,

}remote_mode;

void chassisMain();
void init_chassis();
void chassisDebug();
void kill_chassis();

void update_channel_data(communication::can_bridge_ext_id_t ext_id,
                         communication::can_bridge_data_t data, void* args);

extern control::Chassis* chassis;
extern driver::MotorCANBase* fl_motor;
extern driver::MotorCANBase* fr_motor;
extern driver::MotorCANBase* bl_motor;
extern driver::MotorCANBase* br_motor;
extern float chassis_vx;
extern float chassis_vy;
extern float chassis_vz;
extern bool chassis_boost_flag;