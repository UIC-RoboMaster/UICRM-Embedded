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

#include <string.h>

#include "imu_task.h"
#include "minipc_task.h"

remote::DBUS* dbus = nullptr;
RemoteMode remote_mode = REMOTE_MODE_FOLLOW;
RemoteMode last_remote_mode = REMOTE_MODE_FOLLOW;
RemoteMode available_remote_mode[] = {REMOTE_MODE_FOLLOW, REMOTE_MODE_SPIN, REMOTE_MODE_ADVANCED,
                                      REMOTE_MODE_AUTOPILOT};
const int8_t remote_mode_max = 4;
const int8_t remote_mode_min = 1;
ShootFricMode shoot_flywheel_mode = SHOOT_FRIC_MODE_STOP;
ShootMode shoot_load_mode = SHOOT_MODE_STOP;
CapMode cap_mode = CAP_MODE_CLOSE;
bool is_killed = false;
bool turbo_shoot = false;

void init_dbus() {
    dbus = new remote::DBUS(&huart3);
}
osThreadId_t remoteTaskHandle;
void remoteTask(void* arg) {
    UNUSED(arg);
    osDelay(1000);
    bool mode_switch = false;
    bool shoot_fric_switch = false;
    bool shoot_switch = false;
    bool shoot_burst_switch = false;
    bool shoot_stop_switch = false;
    bool cap_switch = false;
    uint32_t shoot_burst_timestamp = 0;
    remote::switch_t last_state_r = remote::MID;
    remote::switch_t last_state_l = remote::MID;
    remote::keyboard_t last_keyboard;
    remote::mouse_t last_mouse;
    remote::switch_t state_r = remote::MID;
    remote::switch_t state_l = remote::MID;
    remote::keyboard_t keyboard;
    remote::mouse_t mouse;
    memset(&keyboard, 0, sizeof(keyboard));
    memset(&mouse, 0, sizeof(mouse));
    memset(&last_keyboard, 0, sizeof(last_keyboard));
    memset(&last_mouse, 0, sizeof(last_mouse));
    bool is_dbus_offline;
    bool is_robot_dead;
    bool is_shoot_available;
    BoolEdgeDetector* z_edge = new BoolEdgeDetector(false);
    BoolEdgeDetector* shift_edge = new BoolEdgeDetector(false);
    BoolEdgeDetector* mouse_left_edge = new BoolEdgeDetector(false);
    BoolEdgeDetector* mouse_right_edge = new BoolEdgeDetector(false);
    BoolEdgeDetector* shoot_burst_edge = new BoolEdgeDetector(false);
    while (true) {
        // 检测遥控器是否离线，或者遥控器是否在安全模式下
        is_dbus_offline = (!dbus->IsOnline()) || dbus->swr == remote::DOWN;
#ifdef HAS_REFEREE
        // Kill Detection
        is_robot_dead = referee->game_robot_status.remain_HP == 0;
        is_shoot_available = (referee->game_robot_status.shooter_heat_limit -
                              referee->power_heat_data.shooter_id1_17mm_cooling_heat) >= 100 &&
                             referee->bullet_remaining.bullet_remaining_num_17mm > 0 &&
                             imu->CaliDone();
#else
        is_robot_dead = false;
        is_shoot_available = true;
#endif
        if (is_dbus_offline || is_robot_dead) {
            if (!is_killed) {
                // 如果遥控器离线或者机器人死亡，则进入安全模式
                last_remote_mode = remote_mode;
                remote_mode = REMOTE_MODE_KILL;
                shoot_load_mode = SHOOT_MODE_DISABLE;
                is_killed = true;
            }
        } else {
            if (is_killed) {
                // 如果遥控器重新连接或者机器人复活，则恢复上一次的遥控模式
                remote_mode = last_remote_mode;
                shoot_load_mode = SHOOT_MODE_STOP;
                shoot_fric_switch = SHOOT_FRIC_MODE_STOP;
                is_killed = false;
            }
        }
        // when in kill
        if (is_killed) {
            osDelay(REMOTE_OS_DELAY);
            continue;
        }

        // Update Last State
        last_state_r = state_r;
        last_state_l = state_l;
        last_keyboard = keyboard;
        last_mouse = mouse;
        // Update State
        if (dbus->IsOnline()) {
            state_r = dbus->swr;
            state_l = dbus->swl;
            keyboard = dbus->keyboard;
            mouse = dbus->mouse;
        } else if (refereerc->IsOnline()) {
            state_r = remote::MID;
            state_l = remote::MID;
            keyboard = refereerc->remote_control.keyboard;
            mouse = refereerc->remote_control.mouse;
        }

        // Update Timestamp
        z_edge->input(keyboard.bit.Z);
        shift_edge->input(keyboard.bit.SHIFT);
        mouse_left_edge->input(mouse.l);
        mouse_right_edge->input(mouse.r);

        shoot_burst_edge->input(keyboard.bit.F);
        if (shoot_burst_edge->posEdge()) {
            turbo_shoot = !turbo_shoot;
        }

        // remote mode switch
        switch (state_r) {
            case remote::UP:
                if (last_state_r == remote::MID && dbus->IsOnline()) {
                    mode_switch = true;
                }
                break;
            case remote::MID:
                if (shift_edge->posEdge()) {
                    mode_switch = true;
                }
                break;
            case remote::DOWN:
                break;
        }
        if (mode_switch) {
            mode_switch = false;
            RemoteMode next_mode = (RemoteMode)(remote_mode + 1);
//            if (next_mode == RemoteMode::REMOTE_MODE_AUTOPILOT && !minipc->IsOnline())
//                next_mode = (RemoteMode)(next_mode + 1);
            if ((int8_t)next_mode > (int8_t)remote_mode_max) {
                next_mode = (RemoteMode)remote_mode_min;
            }
            remote_mode = next_mode;
        }
        // shoot mode switch
        switch (state_l) {
            case remote::UP:
                if (last_state_l == remote::MID && dbus->IsOnline()) {
                    shoot_fric_switch = true;
                }
                break;
            case remote::DOWN:
                if (last_state_l == remote::MID && dbus->IsOnline() && shoot_flywheel_mode == SHOOT_FRIC_MODE_STOP) {
                    cap_switch = true;
                }
                else if (last_state_l == remote::MID && dbus->IsOnline()) {
                    shoot_switch = true;
                    shoot_burst_timestamp = 0;
                } else if (last_state_l == remote::DOWN && dbus->IsOnline()) {
                    shoot_burst_timestamp++;
                    if (shoot_burst_timestamp > 300 * REMOTE_OS_DELAY) {
                        shoot_burst_switch = true;
                    }
                }
                break;
            case remote::MID:
                switch (last_state_l) {
                    case remote::DOWN:
                        shoot_stop_switch = true;

                        // reset burst states
                        shoot_burst_timestamp = 0;
                        shoot_burst_switch = false;
                        break;
                    case remote::UP:
                        break;
                    case remote::MID:
                        if (z_edge->posEdge()) {
                            shoot_fric_switch = true;
                        }
                        if (mouse_left_edge->posEdge()) {
                            shoot_switch = true;
                            //                                shoot_burst_timestamp = 0;
                        } else if (mouse_left_edge->get()) {
                            //                                shoot_burst_timestamp++;
                            //                                if (shoot_burst_timestamp > 500 *
                            //                                REMOTE_OS_DELAY) {
                            //                                    shoot_burst_switch = true;
                            //                                }
                        } else if (mouse_left_edge->negEdge()) {
                            shoot_stop_switch = true;
                        } else {
                            shoot_stop_switch = true;
                        }
                        break;
                }
                break;
        }
        // 切换摩擦轮模式
        if (shoot_fric_switch) {
            shoot_fric_switch = false;
            if (shoot_flywheel_mode == SHOOT_FRIC_MODE_STOP) {  // 原来停止则开始转
                shoot_flywheel_mode = SHOOT_FRIC_MODE_PREPARING;
                shoot_load_mode = SHOOT_MODE_PREPARING;
            } else if (shoot_flywheel_mode == SHOOT_FRIC_MODE_PREPARED ||
                       shoot_flywheel_mode == SHOOT_FRIC_MODE_PREPARING) {  // 原来转则停止
                shoot_flywheel_mode = SHOOT_FRIC_MODE_STOP;
                shoot_load_mode = SHOOT_MODE_STOP;
            }
        }

        // 射出单颗子弹
        if (shoot_switch) {
            shoot_switch = false;
            if (shoot_load_mode == SHOOT_MODE_PREPARED &&
                shoot_flywheel_mode == SHOOT_FRIC_MODE_PREPARED &&
                (is_shoot_available || SHOOT_REFEREE == 0 || turbo_shoot)) {
                // 摩擦轮与拔弹系统准备就绪则发射子弹
                shoot_load_mode = SHOOT_MODE_SINGLE;
            }
        }

        // 射出连发子弹
        if (shoot_burst_switch) {
            shoot_burst_switch = false;
            if (shoot_flywheel_mode == SHOOT_FRIC_MODE_PREPARED &&
                (shoot_load_mode == SHOOT_MODE_PREPARED || shoot_load_mode == SHOOT_MODE_SINGLE)) {
                // 必须要在准备就绪或者发出单发子弹的情况下才能发射连发子弹
                shoot_load_mode = SHOOT_MODE_BURST;
            }
        }

        // 停止射击
        if (shoot_stop_switch) {
            shoot_stop_switch = false;
            shoot_load_mode = SHOOT_MODE_PREPARED;
        }

        //子弹盖状态
        if (cap_switch) {
            if (cap_mode == CAP_MODE_CLOSE)
                cap_mode = CAP_MODE_OPEN;
            else
                cap_mode = CAP_MODE_CLOSE;
        }

        osDelay(REMOTE_OS_DELAY);
    }
}
void init_remote() {
    init_dbus();
}