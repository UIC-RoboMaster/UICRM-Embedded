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
#include "cmsis_os2.h"
#include "dbus.h"
#include "main.h"
#include "referee_task.h"
#include "user_define.h"
#include "utils.h"
extern remote::DBUS* dbus;
/**
 * @brief Dbus Init
 */
void init_dbus();

enum RemoteMode {
    REMOTE_MODE_PREPARE = -2,
    REMOTE_MODE_KILL = -1,
    REMOTE_MODE_STOP = 0,
    REMOTE_MODE_FOLLOW = 1,
    REMOTE_MODE_SPIN = 2,
    REMOTE_MODE_ADVANCED = 3,
    REMOTE_MODE_AUTOAIM = 4,
};
inline const char* remote_mode_str(RemoteMode mode) {
    switch (mode) {
        case REMOTE_MODE_PREPARE: return "PREPARE"; break;
        case REMOTE_MODE_KILL: return "KILL"; break;
        case REMOTE_MODE_STOP: return "STOP"; break;
        case REMOTE_MODE_FOLLOW: return "FOLLOW"; break;
        case REMOTE_MODE_SPIN: return "SPIN"; break;
        case REMOTE_MODE_ADVANCED: return "ADVANCED"; break;
        case REMOTE_MODE_AUTOAIM: return "AUTOAIM"; break;
        default: return "UNKNOWN"; break;
    }
}
extern RemoteMode remote_mode;

enum ShootFricMode {
    SHOOT_FRIC_MODE_DISABLE = -1,
    SHOOT_FRIC_MODE_STOP = 0,
    SHOOT_FRIC_MODE_PREPARING = 1,
    SHOOT_FRIC_MODE_PREPARED = 2,
};
inline const char* shoot_fric_mode_str(ShootFricMode mode) {
    switch (mode) {
        case SHOOT_FRIC_MODE_DISABLE: return "DISABLE"; break;
        case SHOOT_FRIC_MODE_STOP: return "STOP"; break;
        case SHOOT_FRIC_MODE_PREPARING: return "PREPARING"; break;
        case SHOOT_FRIC_MODE_PREPARED: return "PREPARED"; break;
        default: return "UNKNOWN"; break;
    }
}
extern ShootFricMode shoot_flywheel_mode;

enum ShootMode {
    SHOOT_MODE_DISABLE = -1,
    SHOOT_MODE_STOP = 0,    // 停止供弹
    SHOOT_MODE_IDLE = 1,    // 连续供弹直到检测到弹丸
    SHOOT_MODE_SINGLE = 2,  // 单发，用于通知shoot_task，发射后由shoot_task设置回IDLE
    SHOOT_MODE_BURST = 3,   // 连发
};
inline const char* shoot_load_mode_str(ShootMode mode) {
    switch (mode) {
        case SHOOT_MODE_DISABLE: return "DISABLE"; break;
        case SHOOT_MODE_STOP: return "STOP"; break;
        case SHOOT_MODE_IDLE: return "IDLE"; break;
        case SHOOT_MODE_SINGLE: return "SINGLE"; break;
        case SHOOT_MODE_BURST: return "BURST"; break;
        default: return "UNKNOWN"; break;
    }
}
extern ShootMode shoot_load_mode;

enum ShootSpeed { SHOOT_FREQUENCY_NORMAL = 1, SHOOT_FREQUENCY_FAST = 2, SHOOT_FREQUENCY_BURST = 3 };
extern ShootSpeed shoot_speed;

extern bool is_autoaim;

extern osThreadId_t remoteTaskHandle;
const osThreadAttr_t remoteTaskAttribute = {.name = "remoteTask",
                                            .attr_bits = osThreadDetached,
                                            .cb_mem = nullptr,
                                            .cb_size = 0,
                                            .stack_mem = nullptr,
                                            .stack_size = 768 * 4,
                                            .priority = (osPriority_t)osPriorityHigh,
                                            .tz_module = 0,
                                            .reserved = 0};
void remoteTask(void* arg);
void init_remote();