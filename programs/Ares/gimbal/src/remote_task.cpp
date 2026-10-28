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

controller::DBUS* dbus = nullptr;
RemoteMode remote_mode = REMOTE_MODE_FOLLOW;
RemoteMode last_remote_mode = REMOTE_MODE_FOLLOW;
RemoteMode available_remote_mode[] = {REMOTE_MODE_FOLLOW, REMOTE_MODE_SPIN, REMOTE_MODE_ADVANCED};
const int8_t remote_mode_max = 2;
const int8_t remote_mode_min = 1;

// 摩擦轮状态
ShootFricMode shoot_flywheel_mode = SHOOT_FRIC_MODE_STOP;
ShootMode shoot_load_mode = SHOOT_MODE_STOP;

//
bool is_killed = false;
bool turbo_shoot = false;

// dbus
void init_dbus() {
    dbus = new controller::DBUS(&huart3);
}

osThreadId_t remoteTaskHandle;

// main loop
void remoteTask(void* arg) {
    using enum controller::ThreePositionSwitch;

    // 接口转换
    UNUSED(arg);
    osDelay(1000);
    bool mode_switch = false;
    bool shoot_fric_switch = false;

    // 单发、连发
    bool shoot_switch = false;
    //    bool shoot_burst_switch = false;
    bool shoot_stop_switch = false;
    //    uint32_t shoot_burst_timestamp = 0;

    controller::ThreePositionSwitch last_state_r = Mid;
    controller::ThreePositionSwitch last_state_l = Mid;
    controller::KeyboardState last_keyboard{};
    controller::MouseState last_mouse{};
    controller::ThreePositionSwitch state_r = Mid;
    controller::ThreePositionSwitch state_l = Mid;
    controller::KeyboardState keyboard{};
    controller::MouseState mouse{};

    // 键鼠状态清零

    //
    bool is_dbus_offline;
    bool is_vt13_offline;
    bool is_robot_dead;
    bool is_shoot_available;

    // 键鼠边沿检测
    BoolEdgeDetector* z_edge = new BoolEdgeDetector(false);
    BoolEdgeDetector* shift_edge = new BoolEdgeDetector(false);
    BoolEdgeDetector* mouse_left_edge = new BoolEdgeDetector(false);
    BoolEdgeDetector* mouse_right_edge = new BoolEdgeDetector(false);
    BoolEdgeDetector* shoot_burst_edge = new BoolEdgeDetector(false);
    // VT13 边沿检测器
    BoolEdgeDetector* vt13_pause_edge = new BoolEdgeDetector(false);
    BoolEdgeDetector* vt13_left_edge = new BoolEdgeDetector(false);
    BoolEdgeDetector* vt13_trigger_edge = new BoolEdgeDetector(false);

    while (1) {
        controller::DbusFrame dbus_state{};
        dbus->GetState(dbus_state);
        controller::RefereeRcFrame refereerc_state{};
        if (refereerc != nullptr) {
            refereerc->GetState(refereerc_state);
        }

        const bool vt13_c_mode =
            (refereerc != nullptr && refereerc->IsOnline()) && (refereerc_state.mode == controller::RefereeRcMode::C);

        // Offline Detection && Security Check
        is_dbus_offline = (!dbus->IsOnline()) || dbus_state.right_switch == Down;
        // VT13必须在线且处于C模式才算可控
        is_vt13_offline = !vt13_c_mode;
#ifdef HAS_REFEREE
        is_robot_dead = referee->game_robot_status.remain_HP == 0;
        is_shoot_available = (referee->game_robot_status.shooter_heat_limit -
                              referee->power_heat_data.shooter_id1_42mm_cooling_heat) >= 100 &&
                             imu->CaliDone();
#else
        is_robot_dead = false;
        is_shoot_available = true;
#endif
        // DBUS和VT13都离线，或者机器人死亡，才进入安全模式
        if ((is_dbus_offline && is_vt13_offline) || is_robot_dead) {
            if (!is_killed) {
                last_remote_mode = remote_mode;        // store last mode
                remote_mode = REMOTE_MODE_KILL;        // killed mode
                shoot_load_mode = SHOOT_MODE_DISABLE;  // stop shoot
                shoot_flywheel_mode = SHOOT_FRIC_MODE_STOP;
                is_killed = true;
            }
        } else {  // 复活
            if (is_killed) {
                remote_mode = last_remote_mode;     // return
                shoot_load_mode = SHOOT_MODE_STOP;  // reset shoot mode
                shoot_flywheel_mode = SHOOT_FRIC_MODE_STOP;
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
        state_r = Mid;
        state_l = Mid;
        keyboard = {};
        mouse = {};

        // Update State
        if (dbus->IsOnline()) {  // DBUS
            state_r = dbus_state.right_switch;
            state_l = dbus_state.left_switch;
            keyboard = dbus_state.keyboard;
            mouse = dbus_state.mouse;
        } else if (vt13_c_mode) {
            // referee control use keyboard & mouse
            state_r = Mid;
            state_l = Mid;
            keyboard = refereerc_state.keyboard;
            mouse = refereerc_state.mouse;
        }

        // Update Timestamp
        z_edge->input(keyboard.bit.Z);
        shift_edge->input(keyboard.bit.SHIFT);
        mouse_left_edge->input(mouse.left);
        mouse_right_edge->input(mouse.right);

        shoot_burst_edge->input(keyboard.bit.F);

        if (shoot_burst_edge->posEdge()) {
            turbo_shoot = !turbo_shoot;
        }

        // VT13 边沿检测（仅在VT13可控时有效，避免离线脏数据触发）
        vt13_pause_edge->input(vt13_c_mode && refereerc_state.pause);
        vt13_left_edge->input(vt13_c_mode && refereerc_state.left_button);
        vt13_trigger_edge->input(vt13_c_mode && refereerc_state.trigger);

        // ========== 模式切换 ==========
        // DT7 右摇杆上拨
        switch (state_r) {
            case Up:
                if (last_state_r == Mid && dbus->IsOnline()) {
                    mode_switch = true;
                }
                break;
            case Mid:
                // 键盘 SHIFT 切换模式
                if (shift_edge->posEdge()) {
                    mode_switch = true;
                }
                break;
            case Down:
                break;
        }
        // VT13 暂停键切换模式
        if (vt13_pause_edge->posEdge()) {
            mode_switch = true;
        }
        if (mode_switch) {
            mode_switch = false;
            RemoteMode next_mode = (RemoteMode)(remote_mode + 1);
            if ((int8_t)next_mode > (int8_t)remote_mode_max) {
                next_mode = (RemoteMode)remote_mode_min;
            }
            remote_mode = next_mode;
        }

        // shoot mode switch
        switch (state_l) {
            case Up:
                if (last_state_l == Mid && dbus->IsOnline()) {
                    shoot_fric_switch = true;
                }
                break;
            case Down:
                if (last_state_l == Mid && dbus->IsOnline()) {
                    shoot_switch = true;
                    // shoot_burst_timestamp = 0;
                }
                // else if (last_state_l == Down && selftest.dbus) {
                //     shoot_burst_timestamp++;
                //     if (shoot_burst_timestamp > 500 * REMOTE_OS_DELAY) {
                //         shoot_burst_switch = true;
                //     }
                // }
                break;
            case Mid:
                switch (last_state_l) {
                    case Down:
                        shoot_stop_switch = true;
                        break;
                    case Up:
                        break;
                    case Mid:
                        // 键盘 Z键 切换摩擦轮
                        if (z_edge->posEdge()) {
                            shoot_fric_switch = true;
                        }
                        // 鼠标左键 射击
                        if (mouse_left_edge->posEdge()) {
                            shoot_switch = true;
                            // shoot_burst_timestamp = 0;
                        } else if (mouse_left_edge->get()) {
                            // shoot_burst_timestamp++;
                            // if (shoot_burst_timestamp > 500 *
                            // REMOTE_OS_DELAY) {
                            //     shoot_burst_switch = true;
                            // }
                        } else if (mouse_left_edge->negEdge()) {
                            shoot_stop_switch = true;
                        } else if (!mouse_left_edge->get()) {
                            shoot_stop_switch = true;
                        }
                        break;
                }
                break;
        }
        // VT13 左键切换摩擦轮
        if (vt13_left_edge->posEdge()) {
            shoot_fric_switch = true;
        }
        // VT13 扳机键射击
        if (vt13_trigger_edge->posEdge()) {
            shoot_switch = true;
        }
        if (vt13_trigger_edge->negEdge()) {
            shoot_stop_switch = true;
        }

        // 切换摩擦轮模式
        if (shoot_fric_switch) {
            shoot_fric_switch = false;
            if (shoot_flywheel_mode == SHOOT_FRIC_MODE_STOP) {
                // 原来停止则开始转
                shoot_flywheel_mode = SHOOT_FRIC_MODE_PREPARING;
                shoot_load_mode = SHOOT_MODE_PREPARING;
            } else if (shoot_flywheel_mode == SHOOT_FRIC_MODE_PREPARED ||
                       shoot_flywheel_mode == SHOOT_FRIC_MODE_PREPARING) {
                // 原来转则停止
                shoot_flywheel_mode = SHOOT_FRIC_MODE_STOP;
                shoot_load_mode = SHOOT_MODE_STOP;
            }
        }

        if (shoot_flywheel_mode == SHOOT_FRIC_MODE_STOP && vt13_c_mode) {
            if (refereerc_state.channels[4] < -600) {
                shoot_load_mode = SHOOT_MODE_UNLOAD;
            } else if (refereerc_state.channels[4] > 600) {
                shoot_load_mode = SHOOT_MODE_STOP;
            } else {
                if (refereerc_state.channels[4] > -224 && refereerc_state.channels[4] < 224) {
                    shoot_load_mode = SHOOT_MODE_STOP;
                }
            }
        }

        // single mode
        if (shoot_switch) {
            shoot_switch = false;
            if (shoot_load_mode == SHOOT_MODE_PREPARED && shoot_flywheel_mode == SHOOT_FRIC_MODE_PREPARED &&
                (is_shoot_available || SHOOT_REFEREE == 0 || turbo_shoot)) {
                // 摩擦轮与拔弹系统准备就绪则发射子弹
                shoot_load_mode = SHOOT_MODE_SINGLE;
            }
        }
        // 射出连发子弹
        // if (shoot_burst_switch) {
        //     if (shoot_fric_mode == SHOOT_FRIC_MODE_PREPARED) {
        //         // 必须要在准备就绪或者发出单发子弹的情况下才能发射连发子弹
        //         if (shoot_mode == SHOOT_MODE_PREPARED || shoot_mode ==
        //         SHOOT_MODE_SINGLE) {
        //             shoot_mode = SHOOT_MODE_BURST;
        //             shoot_burst_switch = false;
        //         }
        //     } else {
        //         shoot_burst_switch = false;
        //     }
        // }

        // stop shoot
        if (shoot_stop_switch) {
            shoot_stop_switch = false;
            if (shoot_load_mode == SHOOT_MODE_BURST) {
                shoot_load_mode = SHOOT_MODE_PREPARED;
            }
        }

        osDelay(REMOTE_OS_DELAY);
    }
}
void init_remote() {
    init_dbus();
}