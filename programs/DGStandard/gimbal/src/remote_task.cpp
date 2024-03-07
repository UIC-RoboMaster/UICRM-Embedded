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
RemoteMode remote_mode = REMOTE_MODE_ADVANCED;
RemoteMode last_remote_mode = REMOTE_MODE_ADVANCED;
RemoteMode available_remote_mode[] = {REMOTE_MODE_FOLLOW, REMOTE_MODE_SPIN, REMOTE_MODE_ADVANCED};
const int8_t remote_mode_max = 3;
const int8_t remote_mode_min = 1;
ShootFricMode shoot_flywheel_mode = SHOOT_FRIC_MODE_STOP;
ShootMode shoot_feed_mode = SHOOT_MODE_STOP;
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
    bool is_dbus_offline;
    bool is_robot_dead;
    bool is_shoot_available;

    BoolEdgeDetector* keyboard_Z_edge = new BoolEdgeDetector(false);
    BoolEdgeDetector* keyboard_ctrl_edge = new BoolEdgeDetector(false);
    BoolEdgeDetector* mouse_left_edge = new BoolEdgeDetector(false);
    BoolEdgeDetector* mouse_right_edge = new BoolEdgeDetector(false);
    BoolEdgeDetector* keyboard_G_edge = new BoolEdgeDetector(false);
    BoolEdgeDetector* keyboard_B_edge = new BoolEdgeDetector(false);

    while (1) {
        // Offline Detection && Security Check
        is_dbus_offline = (!selftest.dbus) || dbus->swr == remote::DOWN;
        // Kill Detection
        //        is_robot_dead = referee->game_robot_status.remain_HP == 0;
        //        is_shoot_available =
        //            referee->bullet_remaining.bullet_remaining_num_17mm > 0 && imu->CaliDone();
        is_robot_dead = false;
        is_shoot_available = true;
        if (is_dbus_offline || is_robot_dead) {
            if (!is_killed) {
                last_remote_mode = remote_mode;
                remote_mode = REMOTE_MODE_KILL;
                shoot_feed_mode = SHOOT_MODE_DISABLE;
                is_killed = true;
            }
        } else {
            if (is_killed) {
                remote_mode = last_remote_mode;
                shoot_feed_mode = SHOOT_MODE_STOP;
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

        // Update Timestamp
        mouse_left_edge->input(mouse.l);
        mouse_right_edge->input(mouse.r);
        keyboard_ctrl_edge->input(keyboard.bit.CTRL);

        keyboard_G_edge->input(keyboard.bit.G);
        keyboard_B_edge->input(keyboard.bit.B);

        if (keyboard_G_edge->posEdge()) {
            shoot_flywheel_mode = SHOOT_FRIC_SPEEDUP;
        }
        if (keyboard_B_edge->posEdge()) {
            shoot_flywheel_mode = SHOOT_FRIC_SPEEDDOWN;
        }

        // remote mode switch
        static BoolEdgeDetector* mode_switch_edge = new BoolEdgeDetector(false);
        mode_switch_edge->input(state_r == remote::UP);

        if (mode_switch_edge->posEdge() || keyboard_ctrl_edge->posEdge()){
            RemoteMode next_mode = (RemoteMode)(remote_mode + 1);
            if ((int8_t)next_mode > (int8_t)remote_mode_max) {
                next_mode = (RemoteMode)remote_mode_min;
            }
            remote_mode = next_mode;
        }

        if (!is_shoot_available)
        {
            shoot_feed_mode = SHOOT_MODE_STOP;
            shoot_flywheel_mode = SHOOT_FRIC_MODE_STOP;
        }

        // 切换摩擦轮
        static BoolEdgeDetector* flywheel_switch_edge = new BoolEdgeDetector(false);
        flywheel_switch_edge->input(state_l == remote::UP);
        keyboard_Z_edge->input(keyboard.bit.Z);
        if (flywheel_switch_edge->posEdge() ||
            keyboard_Z_edge->posEdge()) {

            if (shoot_flywheel_mode == SHOOT_FRIC_MODE_STOP) {  // 原来停止则开始转
                shoot_flywheel_mode = SHOOT_FRIC_MODE_PREPARING;
                shoot_feed_mode = SHOOT_MODE_PREPARING;
            } else if (shoot_flywheel_mode == SHOOT_FRIC_MODE_PREPARING ||
                       shoot_flywheel_mode == SHOOT_FRIC_MODE_PREPARED) {  // 原来转则停止
                shoot_flywheel_mode = SHOOT_FRIC_MODE_STOP;
                shoot_feed_mode = SHOOT_MODE_STOP;
            }
        }

        // 单发
        static BoolEdgeDetector* shoot_switch_edge = new BoolEdgeDetector(false);
        shoot_switch_edge->input(state_l == remote::DOWN);
        if (shoot_switch_edge->posEdge()) {
            shoot_feed_mode = SHOOT_MODE_SINGLE;
            shoot_burst_timestamp = 0;
        }

        // 连发
        if (state_l == remote::DOWN) {
            shoot_burst_timestamp++;
        }
        static BoolEdgeDetector* shoot_burst_switch_edge = new BoolEdgeDetector(false);
        shoot_burst_switch_edge->input(shoot_burst_timestamp > 500 * REMOTE_OS_DELAY);
        if (shoot_burst_switch_edge->posEdge()) {
            shoot_feed_mode = SHOOT_MODE_BURST;
        }

        // 不发射
        if (shoot_switch_edge->negEdge()) {
            shoot_feed_mode = SHOOT_MODE_STOP;
            shoot_burst_timestamp = 0;
        }
        osDelay(REMOTE_OS_DELAY);
    }
}
void init_remote() {
    init_dbus();
}