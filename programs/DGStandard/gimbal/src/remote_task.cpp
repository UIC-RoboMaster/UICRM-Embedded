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

#include "remote_task.h"

#include "bsp_os.h"

remote::DBUS* dbus = nullptr;
RemoteMode remote_mode = REMOTE_MODE_KILL;
RemoteMode last_remote_mode = REMOTE_MODE_FOLLOW;
RemoteMode available_remote_mode[] = {REMOTE_MODE_FOLLOW, REMOTE_MODE_SPIN, REMOTE_MODE_ADVANCED, REMOTE_MODE_AUTOAIM};
const int8_t remote_mode_max = 2;
const int8_t remote_mode_min = 1;
ShootFricMode shoot_flywheel_mode = SHOOT_FRIC_MODE_STOP;
ShootMode shoot_load_mode = SHOOT_MODE_STOP;

ShootSpeed shoot_speed = SHOOT_FREQUENCY_NORMAL;
const int8_t shoot_speed_min = 1;
const int8_t shoot_speed_max = 2;

bool is_killed = true;

bool is_autoaim = false;

void init_dbus() {
    // 初始化遥控器
    dbus = new remote::DBUS(&huart3);
}
osThreadId_t remoteTaskHandle;

// #define HAS_REFEREE

void remoteTask(void* arg) {
    UNUSED(arg);
    bool shoot_controls_armed = false;
    remote::switch_t last_left = remote::MID;
    uint32_t down_started_ms = 0;

    while (true) {
        const bool enabled = dbus->IsOnline() &&
                             (dbus->swr == remote::UP || dbus->swr == remote::MID);
        is_killed = !enabled;
        remote_mode = enabled ? REMOTE_MODE_FOLLOW : REMOTE_MODE_KILL;
        is_autoaim = false;

        if (!enabled) {
            shoot_flywheel_mode = SHOOT_FRIC_MODE_STOP;
            shoot_load_mode = SHOOT_MODE_DISABLE;
            shoot_controls_armed = false;
        } else if (!shoot_controls_armed) {
            // 每次解锁后左拨杆先回中，避免下挡开机或重连后立即供弹。
            shoot_flywheel_mode = SHOOT_FRIC_MODE_STOP;
            shoot_load_mode = SHOOT_MODE_STOP;
            if (dbus->swl == remote::MID) {
                shoot_controls_armed = true;
                last_left = remote::MID;
            }
        } else {
            const remote::switch_t left = dbus->swl;
            const uint32_t now_ms = bsp::GetHighresTickMilliSec();
            if (left == remote::UP && last_left != remote::UP) {
                shoot_flywheel_mode = shoot_flywheel_mode == SHOOT_FRIC_MODE_STOP
                                          ? SHOOT_FRIC_MODE_PREPARING : SHOOT_FRIC_MODE_STOP;
                shoot_load_mode = SHOOT_MODE_STOP;
            }
            if (left == remote::DOWN) {
                if (last_left != remote::DOWN) {
                    down_started_ms = now_ms;
                    shoot_load_mode = shoot_flywheel_mode == SHOOT_FRIC_MODE_PREPARED
                                          ? SHOOT_MODE_SINGLE : SHOOT_MODE_STOP;
                } else if (now_ms - down_started_ms >= 200 &&
                           shoot_flywheel_mode == SHOOT_FRIC_MODE_PREPARED) {
                    shoot_load_mode = SHOOT_MODE_BURST;
                }
            } else if (last_left == remote::DOWN) {
                shoot_load_mode = SHOOT_MODE_STOP;
            }
            if (shoot_flywheel_mode == SHOOT_FRIC_MODE_STOP) {
                shoot_load_mode = SHOOT_MODE_STOP;
            }
            last_left = left;
        }
        osDelay(REMOTE_OS_DELAY);
    }
}
void init_remote() {
    init_dbus();
}
