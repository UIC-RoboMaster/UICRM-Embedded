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

remote::DBUS* dbus = nullptr;
RemoteMode remote_mode = REMOTE_MODE_MANUAL;
RemoteMode last_remote_mode = REMOTE_MODE_MANUAL;
RemoteMode available_remote_mode[] = {REMOTE_MODE_FOLLOW, REMOTE_MODE_SPIN, REMOTE_MODE_MANUAL};
const int8_t remote_mode_max = 3;
const int8_t remote_mode_min = 1;
ShootFricMode shoot_flywheel_mode = SHOOT_FRIC_MODE_STOP;
ShootMode shoot_load_mode = SHOOT_MODE_STOP;
bool is_killed = false;

void init_dbus() {
    dbus = new remote::DBUS(&huart1);
}
osThreadId_t remoteTaskHandle;

void remoteTask(void* arg) {
    UNUSED(arg);
    osDelay(1000);
    uint32_t shoot_burst_timestamp = 0;

    remote::switch_t state_r = remote::MID;
    remote::switch_t state_l = remote::MID;
    remote::keyboard_t keyboard;
    remote::mouse_t mouse;

    memset(&keyboard, 0, sizeof(keyboard));
    memset(&mouse, 0, sizeof(mouse));

    while (1) {
        // Offline Detection && Security Check
        bool is_dbus_offline = (!selftest.dbus) || dbus->swr == remote::DOWN;
#ifdef HAS_REFEREE
        // Kill Detection
        bool is_robot_dead = referee->game_robot_status.remain_HP == 0;
        bool is_shoot_available =
            referee->bullet_remaining.bullet_remaining_num_17mm > 0 && imu->CaliDone();
#else
        bool is_robot_dead = false;
        bool is_shoot_available = true;
#endif
        if (is_dbus_offline || is_robot_dead) {
            if (!is_killed) {
                last_remote_mode = remote_mode;
                remote_mode = REMOTE_MODE_KILL;
                shoot_load_mode = SHOOT_MODE_DISABLE;
                is_killed = true;
            }
        } else {
            if (is_killed) {
                remote_mode = last_remote_mode;
                shoot_load_mode = SHOOT_MODE_STOP;
                is_killed = false;
            }
        }

        // when in kill
        if (is_killed) {
            osDelay(REMOTE_OS_DELAY);
            continue;
        }

        // Update State
        if (selftest.dbus) {
            state_r = dbus->swr;
            state_l = dbus->swl;
            keyboard = dbus->keyboard;
            mouse = dbus->mouse;
        } else if (selftest.refereerc) {
            state_r = remote::MID;
            state_l = remote::MID;
            keyboard = refereerc->remote_control.keyboard;
            mouse = refereerc->remote_control.mouse;
        }

        // 更新状态

        // 切换控制模式
        static BoolEdgeDetector* mode_switch_edge = new BoolEdgeDetector(false);
        mode_switch_edge->input(state_r == remote::UP);
        static BoolEdgeDetector* keyboard_ctrl_edge = new BoolEdgeDetector(false);
        keyboard_ctrl_edge->input(keyboard.bit.CTRL);
        if (mode_switch_edge->posEdge() || keyboard_ctrl_edge->posEdge()){
            RemoteMode next_mode = (RemoteMode)(remote_mode + 1);
            if ((int8_t)next_mode > (int8_t)remote_mode_max) {
                next_mode = (RemoteMode)remote_mode_min;
            }
            remote_mode = next_mode;
        }

        /*
         * 射击模式控制
         * */

        if (!is_shoot_available)
        {
            shoot_load_mode = SHOOT_MODE_STOP;
            shoot_flywheel_mode = SHOOT_FRIC_MODE_STOP;
        }

        // 切换摩擦轮
        static BoolEdgeDetector* flywheel_switch_edge = new BoolEdgeDetector(false);
        flywheel_switch_edge->input(state_l == remote::UP);
        static BoolEdgeDetector* keyboard_Z_edge = new BoolEdgeDetector(false);
        keyboard_Z_edge->input(keyboard.bit.Z);
        if (flywheel_switch_edge->posEdge() ||
            keyboard_Z_edge->posEdge()) {

            if (shoot_flywheel_mode == SHOOT_FRIC_MODE_STOP) {  // 原来停止则开始转
                shoot_flywheel_mode = SHOOT_FRIC_MODE_PREPARING;
                shoot_load_mode = SHOOT_MODE_IDLE;
            } else if (shoot_flywheel_mode == SHOOT_FRIC_MODE_PREPARING ||
                       shoot_flywheel_mode == SHOOT_FRIC_MODE_PREPARED) {  // 原来转则停止
                shoot_flywheel_mode = SHOOT_FRIC_MODE_STOP;
                shoot_load_mode = SHOOT_MODE_STOP;
            }
        }

        // 单发
        static BoolEdgeDetector* shoot_switch_edge = new BoolEdgeDetector(false);
        shoot_switch_edge->input(state_l == remote::DOWN);
        static BoolEdgeDetector* mouse_left_edge = new BoolEdgeDetector(false);
        mouse_left_edge->input(mouse.l);
        if (shoot_switch_edge->posEdge() ||
            mouse_left_edge->posEdge()) {
            shoot_load_mode = SHOOT_MODE_SINGLE;
            shoot_burst_timestamp = 0;
        }

        // 连发
        if (state_l == remote::DOWN || mouse.l) {
            shoot_burst_timestamp++;
        }
        static BoolEdgeDetector* shoot_burst_switch_edge = new BoolEdgeDetector(false);
        shoot_burst_switch_edge->input(shoot_burst_timestamp > 500 * REMOTE_OS_DELAY);
        if (shoot_burst_switch_edge->posEdge()) {
            shoot_load_mode = SHOOT_MODE_BURST;
        }

        // 停止发射
        if (shoot_switch_edge->negEdge()) {
            shoot_load_mode = SHOOT_MODE_STOP;
            shoot_burst_timestamp = 0;
        }
        osDelay(REMOTE_OS_DELAY);
    }
}
void init_remote() {
    init_dbus();
}