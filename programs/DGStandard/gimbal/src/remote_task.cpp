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
#include "utils.h"

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

    while (true) {
        const bool enabled = dbus->IsOnline() &&
                             (dbus->swr == remote::UP || dbus->swr == remote::MID);
        is_killed = !enabled;
        remote_mode = enabled ? REMOTE_MODE_FOLLOW : REMOTE_MODE_KILL;
        is_autoaim = false;

        if (!enabled) {
            shoot_flywheel_mode = SHOOT_FRIC_MODE_STOP;
            shoot_load_mode = SHOOT_MODE_DISABLE;
        } else {
            // 原始遥控逻辑：UP=切换摩擦轮（上拨打开、回中、再上拨关闭）；DOWN=单发，按住=连发
            static BoolEdgeDetector* flywheel_edge = new BoolEdgeDetector(false);
            static BoolEdgeDetector* shoot_edge = new BoolEdgeDetector(false);
            static uint32_t shoot_burst_timestamp = 0;
            static BoolEdgeDetector* burst_edge = new BoolEdgeDetector(false);

            flywheel_edge->input(dbus->swl == remote::UP);
            shoot_edge->input(dbus->swl == remote::DOWN);

            // 上拨：切换摩擦轮（原来停止则开始转，原来转则停止）
            if (flywheel_edge->posEdge()) {
                if (shoot_flywheel_mode == SHOOT_FRIC_MODE_STOP) {
                    shoot_flywheel_mode = SHOOT_FRIC_MODE_PREPARING;
                    shoot_load_mode = SHOOT_MODE_IDLE;
                } else {
                    shoot_flywheel_mode = SHOOT_FRIC_MODE_STOP;
                    shoot_load_mode = SHOOT_MODE_STOP;
                }
            }

            // 下拨：单发；按住超过 200ms 转连发；松开停止
            if (shoot_edge->posEdge()) {
                shoot_load_mode = SHOOT_MODE_SINGLE;
                shoot_burst_timestamp = 0;
            }
            if (dbus->swl == remote::DOWN) {
                shoot_burst_timestamp += REMOTE_OS_DELAY;
            }
            burst_edge->input(shoot_burst_timestamp > 200);
            if (burst_edge->posEdge()) {
                shoot_load_mode = SHOOT_MODE_BURST;
            }
            if (shoot_edge->negEdge()) {
                shoot_load_mode = SHOOT_MODE_STOP;
                shoot_burst_timestamp = 0;
            }
        }
        osDelay(REMOTE_OS_DELAY);
    }
}
void init_remote() {
    init_dbus();
}
