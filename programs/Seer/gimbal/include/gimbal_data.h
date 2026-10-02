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
#include "dual_yaw_gimbal.h"
#include "pid.h"

// CAN1: 大 yaw；CAN2: 小 yaw / pitch。
// 暂按 README 的 4/2/6 为 GM6020 拨码 ID，反馈 ID = 0x204 + 拨码 ID。
constexpr uint16_t lower_yaw_rx_id = 0x208;
constexpr uint16_t upper_yaw_rx_id = 0x206;
constexpr uint16_t pitch_rx_id = 0x20A;

// basic information of gimbal
// 零点、关节方向和回中步长需要按本车实测标定，不能直接沿用示例零点。
const control::dual_yaw_gimbal_data_t gimbal_init_data = {
    .pitch_offset_ = 1.8f,
    .pitch_max_ = 0.595F,
    .pitch_inverted = true,
    .pitch_deadband = 0,

    .upper_yaw_offset_ = 3.0572f,
    .upper_yaw_max_ = PI / 2,
    .upper_yaw_circle_ = false,
    .upper_yaw_inverted = false,
    .upper_yaw_deadband = 0,

    .lower_yaw_offset_ = 3.7f,
    .lower_yaw_max_ = PI,
    .lower_yaw_circle_ = true,
    .lower_yaw_inverted = false,
    .lower_yaw_deadband = 0,

    .upper_yaw_joint_inverted = false,
    .lower_yaw_joint_inverted = false,
    .lower_yaw_recenter_max_step = 0.02F,
};
