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

remote::SBUS* sbus = nullptr;
RemoteMode remote_mode = REMOTE_MODE_FOLLOW;
RemoteMode last_remote_mode = REMOTE_MODE_FOLLOW;
RemoteMode available_remote_mode[] = {REMOTE_MODE_FOLLOW, REMOTE_MODE_SPIN, REMOTE_MODE_ADVANCED};
const int8_t remote_mode_max = 3;
const int8_t remote_mode_min = 1;
ShootFricMode shoot_flywheel_mode = SHOOT_FRIC_MODE_STOP;
ShootMode shoot_load_mode = SHOOT_MODE_STOP;
bool is_killed = false;

void init_dbus() {
    sbus = new remote::SBUS(&huart3);
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
    uint32_t shoot_burst_timestamp = 0;
    int16_t last_state_r = 0;
    int16_t last_state_l = 0;
    int16_t state_r = 0;
    int16_t state_l = 0;
    bool is_dbus_offline;
    bool is_robot_dead;
    bool is_shoot_available;

    BoolEdgeDetector* z_edge = new BoolEdgeDetector(false);
    BoolEdgeDetector* ctrl_edge = new BoolEdgeDetector(false);
    BoolEdgeDetector* mouse_left_edge = new BoolEdgeDetector(false);
    BoolEdgeDetector* mouse_right_edge = new BoolEdgeDetector(false);

    remote::keyboard_t last_keyboard;
    remote::mouse_t last_mouse;
    remote::keyboard_t keyboard;
    remote::mouse_t mouse;

    while (1) {
        // Offline Detection && Security Check
        is_dbus_offline = (!selftest.sbus && !selftest.refereerc) || sbus->ch7 <= -550;
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

        // Update Last State
        last_state_r = state_r;
        last_state_l = state_l;
        last_keyboard = keyboard;
        last_mouse = mouse;
        // Update State
        if (selftest.sbus) {
            state_r = sbus->ch7;
            state_l = sbus->ch8;
        }
        if (selftest.refereerc && state_l == 0 && state_r == 0){
            state_r = 0;
            state_l = 0;
            keyboard = refereerc->remote_control.keyboard;
            mouse = refereerc->remote_control.mouse;
            z_edge->input(keyboard.bit.Z);
            ctrl_edge->input(keyboard.bit.CTRL);
            mouse_left_edge->input(mouse.l);
            mouse_right_edge->input(mouse.r);
        }


        // Update Timestamp

        if (sbus->ch7 > 0 && last_state_r == 0) {
            mode_switch = true;
        }else if (ctrl_edge->posEdge()) {
            mode_switch = true;
        }
        // remote mode switch

        if (mode_switch) {
            mode_switch = false;
            RemoteMode next_mode = (RemoteMode)(remote_mode + 1);
            if ((int8_t)next_mode > (int8_t)remote_mode_max) {
                next_mode = (RemoteMode)remote_mode_min;
            }
            remote_mode = next_mode;
        }
        // shoot mode switch
        if (is_shoot_available == true || SHOOT_REFEREE == 0) {
            if (sbus->ch8 > 0 && last_state_l == 0) {
                shoot_fric_switch = true;
            } else {
                if (sbus->ch8 < 0 && last_state_l == 0) {
                    shoot_switch = true;
                    shoot_burst_timestamp = 0;
                } else if (last_state_l < 0 && selftest.sbus) {
                    shoot_burst_timestamp++;
                    if (shoot_burst_timestamp > 500 * REMOTE_OS_DELAY) {
                        shoot_burst_switch = true;
                    }
                } else {
                    if(selftest.refereerc){
                        if (z_edge->posEdge()) {
                            shoot_fric_switch = true;
                        }
                        if (mouse_left_edge->posEdge()) {
                            shoot_switch = true;
                            shoot_burst_timestamp = 0;
                        } else if (mouse_left_edge->get()) {
                            shoot_burst_timestamp++;
                            if (shoot_burst_timestamp > 500 * REMOTE_OS_DELAY) {
                                shoot_burst_switch = true;
                            }
                        } else if (mouse_left_edge->negEdge()) {
                            shoot_stop_switch = true;
                        } else {
                            shoot_stop_switch = true;
                        }
                    }

                }
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
                    shoot_flywheel_mode == SHOOT_FRIC_MODE_PREPARED) {
                    // 摩擦轮与拔弹系统准备就绪则发射子弹
                    shoot_load_mode = SHOOT_MODE_SINGLE;
                }
            }
            // 射出连发子弹
            if (shoot_burst_switch) {
                if (shoot_flywheel_mode == SHOOT_FRIC_MODE_PREPARED) {
                    // 必须要在准备就绪或者发出单发子弹的情况下才能发射连发子弹
                    if (shoot_load_mode == SHOOT_MODE_PREPARED ||
                        shoot_load_mode == SHOOT_MODE_SINGLE) {
                        shoot_load_mode = SHOOT_MODE_BURST;
                        shoot_burst_switch = false;
                    }
                } else {
                    shoot_burst_switch = false;
                }
            }
            // 停止射击
            if (shoot_stop_switch) {
                shoot_stop_switch = false;
                if (shoot_load_mode == SHOOT_MODE_BURST) {
                    shoot_load_mode = SHOOT_MODE_PREPARED;
                }
            }
        } else {
            // 没有子弹了，则停止射击
            shoot_load_mode = SHOOT_MODE_STOP;
            shoot_flywheel_mode = SHOOT_FRIC_MODE_STOP;
        }
        osDelay(REMOTE_OS_DELAY);
    }
}
void init_remote() {
    init_dbus();
}