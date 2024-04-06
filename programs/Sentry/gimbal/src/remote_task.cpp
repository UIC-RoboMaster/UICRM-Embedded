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

#include "chassis_task.h"
#include "gimbal_task.h"
#include "imu_task.h"

remote::DBUS* dbus = nullptr;
RemoteMode remote_mode = REMOTE_MODE_ADVANCED;
RemoteMode last_remote_mode = REMOTE_MODE_ADVANCED;
RemoteMode available_remote_mode[] = {REMOTE_MODE_FOLLOW, REMOTE_MODE_SPIN, REMOTE_MODE_ADVANCED,
                                      REMOTE_MODE_AUTOMATIC};
const int8_t remote_mode_max = 3;
const int8_t remote_mode_min = 3;
ShootFricMode shoot_flywheel_mode = SHOOT_FRIC_MODE_STOP;
ShootMode shoot_load_mode = SHOOT_MODE_STOP;

BoolEdgeDetector* game_start_edge = nullptr;
bool is_chassis_ok = false;

bool is_killed = false;

bool is_autoaim = false;

void init_dbus() {
    // 初始化遥控器
    dbus = new remote::DBUS(&huart3);
}
osThreadId_t remoteTaskHandle;

// #define HAS_REFEREE

void remoteTask(void* arg) {
    UNUSED(arg);
    osDelay(1000);

    remote::switch_t state_r = remote::MID;
    remote::switch_t state_l = remote::MID;
    remote::keyboard_t keyboard;
    remote::mouse_t mouse;

    memset(&keyboard, 0, sizeof(keyboard));
    memset(&mouse, 0, sizeof(mouse));
    bool is_dbus_offline;
    bool is_robot_dead;
    bool is_shoot_available;

    BoolEdgeDetector* keyboard_ctrl_edge = new BoolEdgeDetector(false);
    BoolEdgeDetector* mouse_left_edge = new BoolEdgeDetector(false);
    BoolEdgeDetector* mouse_right_edge = new BoolEdgeDetector(false);
    BoolEdgeDetector* keyboard_G_edge = new BoolEdgeDetector(false);
    BoolEdgeDetector* keyboard_B_edge = new BoolEdgeDetector(false);

    game_start_edge = new BoolEdgeDetector(false);

    while (1) {
        // 检测遥控器是否离线，或者遥控器是否在安全模式下
        is_dbus_offline = (!dbus->IsOnline()) || dbus->swr == remote::DOWN;
#ifdef HAS_REFEREE
        // Kill Detection
        is_robot_dead = referee->game_robot_status.remain_HP == 0;

#else
        is_robot_dead = false;
#endif
#ifdef SHOOT_REFEREE
        is_shoot_available =
            referee->game_robot_status.mains_power_shooter_output!=0;
#else
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
                is_killed = false;
            }
        }

        // when in kill
        if (is_killed) {
            osDelay(REMOTE_OS_DELAY);
            continue;
        }

        if (dbus->IsOnline()) {
            state_r = dbus->swr;
            state_l = dbus->swl;
            keyboard = dbus->keyboard;
            mouse = dbus->mouse;
        }

        // Update Timestamp
        mouse_left_edge->input(mouse.l);
        mouse_right_edge->input(mouse.r);
        keyboard_ctrl_edge->input(keyboard.bit.CTRL);

        keyboard_G_edge->input(keyboard.bit.G);
        keyboard_B_edge->input(keyboard.bit.B);

        game_start_edge->input(referee->game_status.game_progress==4);
        // remote mode switch

        switch(state_r){
            case remote::MID:
                remote_mode = REMOTE_MODE_ADVANCED;
                break;
            case remote::DOWN:
                remote_mode = REMOTE_MODE_KILL;
                break;
            case remote::UP:

                if(is_chassis_ok){
                    remote_mode = REMOTE_MODE_SPIN;
                }else{
                    remote_mode = REMOTE_MODE_FOLLOW;
                }
        }

        /*
         * 射击模式控制
         * shoot_flywheel_mode：设置PREPARING/STOP控制摩擦轮启停，就绪后由shoot_task转为PREPARED
         * shoot_load_mode：设置SINGLE/BURST/STOP切换发射模式（供弹模式）
         * */

        if (!is_shoot_available) {
            shoot_load_mode = SHOOT_MODE_STOP;
            shoot_flywheel_mode = SHOOT_FRIC_MODE_STOP;
        }

        // 切换摩擦轮
        static BoolEdgeDetector* flywheel_switch_edge = new BoolEdgeDetector(false);
        flywheel_switch_edge->input(state_l == remote::UP);
        if (flywheel_switch_edge->posEdge()) {
            if (shoot_flywheel_mode == SHOOT_FRIC_MODE_STOP) {  // 原来停止则开始转
                shoot_flywheel_mode = SHOOT_FRIC_MODE_PREPARING;
                shoot_load_mode = SHOOT_MODE_IDLE;
            } else if (shoot_flywheel_mode == SHOOT_FRIC_MODE_PREPARING ||
                       shoot_flywheel_mode == SHOOT_FRIC_MODE_PREPARED) {  // 原来转则停止
                shoot_flywheel_mode = SHOOT_FRIC_MODE_STOP;
                shoot_load_mode = SHOOT_MODE_STOP;
            }
        }


        if (!is_autoaim || !minipc->IsOnline()) {
            // 单发
            static BoolEdgeDetector* shoot_switch_edge = new BoolEdgeDetector(false);
            shoot_switch_edge->input(state_l == remote::DOWN);

            if (shoot_switch_edge->posEdge() || mouse_left_edge->posEdge()) {
                shoot_load_mode = SHOOT_MODE_BURST;
            }

            // 连发

            // 不发射
            if (shoot_switch_edge->negEdge() || mouse_left_edge->negEdge()) {
                shoot_load_mode = SHOOT_MODE_STOP;
            }
        } else {
            // 自喵模式下只有连发
            if (minipc->IsOnline()) {
                if (minipc->target_angle.shoot_cmd != 0) {
                    shoot_load_mode = SHOOT_MODE_BURST;
                } else {
                    shoot_load_mode = SHOOT_MODE_STOP;
                }
            }
        }
        osDelay(REMOTE_OS_DELAY);
    }
}
void init_remote() {
    init_dbus();
}