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

// this is a config file, every variable will define here
// 这个文件用来统一配置所有的变量

#pragma once

/**
 * @note  Chassis Config
 * @author BNU-HKBU UIC RoboMaster
 * @date   2023-10-01
 * @version 1.0
*/
// motor init
#define CHASSIS_CAN can1
#define FL_MOTOR_ID 0x202
#define FR_MOTOR_ID 0x201
#define BL_MOTOR_ID 0x203
#define BR_MOTOR_ID 0x204

// pid
#define CHASSIS_PID_KP 2500
#define CHASSIS_PID_KI 3
#define CHASSIS_PID_KD 0
#define CHASSIS_PID_FC 0.1

// supercap
#define SUPER_CAP_CAN can1
#define SUPER_CAP_TX_ID 0x02e
#define SUPER_CAP_TX_SETTINGS_ID 0x02f
#define SUPER_CAP_RX_ID 0x030

/**
 * @note  Gimbal Config
 * @author BNU-HKBU UIC RoboMaster
 * @date   2023-10-01
 * @version 1.0
*/
// motor init
#define GIMBAL_CAN can2
#define PITCH_MOTOR_RX_ID 0x208
#define PITCH_MOTOR_TX_ID 0x1FE
#define YAW_MOTOR_RX_ID 0x20A
#define YAW_MOTOR_TX_ID 0x2FE

// pid
#define PITCH_PID_KP 2
#define PITCH_PID_KI 2
#define PITCH_PID_KD 0
#define PITCH_PID_FC 0.1

// Remote Config

// MiniPC Config

// Referee Config

// Buzzer_Config