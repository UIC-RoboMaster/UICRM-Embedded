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
#include "user_define.h"
#include "ui_task.h"

#include <string.h>

#include "gimbal_task.h"
// todo:陀螺仪未上线！
// #include "imu_task.h"

remote::DBUS* dbus = nullptr;
remote::SBUS* sbus = nullptr;

//RemoteMode remote_mode = REMOTE_MODE_SPIN;
//RemoteMode last_remote_mode = REMOTE_MODE_ADVANCED;
RemoteMode remote_mode = REMOTE_MODE_FOLLOW;
RemoteMode last_remote_mode = REMOTE_MODE_AUTOAIM;
RemoteMode available_remote_mode[] = {REMOTE_MODE_FOLLOW, REMOTE_MODE_SPIN, REMOTE_MODE_ADVANCED,
                                     REMOTE_MODE_AUTOAIM, REMOTE_MODE_PREPARE_HAND_MOVEMENT};
const int8_t remote_mode_max = 4;
const int8_t remote_mode_min = 1;
ShootFricMode shoot_flywheel_mode = SHOOT_FRIC_MODE_STOP;
ShootFricMode last_shoot_flywheel_mode = SHOOT_FRIC_MODE_PREPARING;
ShootMode shoot_load_mode = SHOOT_MODE_STOP;
ShootMode last_shoot_load_mode = SHOOT_MODE_RELOADING;

MagazineMode magazine_Mode = Magazine_MODE_OFF;

ShootSpeed shoot_speed = SHOOT_FREQUENCY_NORMAL;
const int8_t shoot_speed_min = 1;
const int8_t shoot_speed_max = 2;

bool is_killed = false;
bool robot_iskill = false;

bool is_autoaim = false;

// todo: 将void init_dbus() 改为 init_controller()
void init_controller() {
   // 初始化遥控器
#ifdef DBUS_MODE
   dbus = new remote::DBUS(&dbus_uart_post);
#endif
#ifdef SBUS_MODE
   sbus = new remote::SBUS(&sbus_uart_post);
#endif
}

osThreadId_t remoteTaskHandle;

// #define HAS_REFEREE

void remoteTask(void* arg) {
   UNUSED(arg);
   osDelay(1000);
   uint32_t shoot_burst_timestamp = 0;

#ifdef DBUS_MODE
   remote::switch_t state_r = remote::MID;
   remote::switch_t state_l = remote::MID;
   remote::keyboard_t keyboard;
   remote::mouse_t mouse;

   memset(&keyboard, 0, sizeof(keyboard));
   memset(&mouse, 0, sizeof(mouse));

   BoolEdgeDetector* keyboard_shift_edge = new BoolEdgeDetector(false);
   BoolEdgeDetector* mouse_left_edge = new BoolEdgeDetector(false);
   BoolEdgeDetector* mouse_right_edge = new BoolEdgeDetector(false);
   BoolEdgeDetector* keyboard_ctrl_edge = new BoolEdgeDetector(false);

   BoolEdgeDetector* keyboard_Z_edge = new BoolEdgeDetector(false);
   BoolEdgeDetector* keyboard_B_edge = new BoolEdgeDetector(false);

   BoolEdgeDetector* keyboard_R_edge = new BoolEdgeDetector(false);
   BoolEdgeDetector* keyboard_V_edge = new BoolEdgeDetector(false);
   BoolEdgeDetector* keyboard_F_edge = new BoolEdgeDetector(false);

   BoolEdgeDetector* keyboard_N_edge = new BoolEdgeDetector(false);
   BoolEdgeDetector* keyboard_X_edge = new BoolEdgeDetector(false);
#endif

   bool is_dbus_offline;
   bool is_robot_dead;
   bool is_shoot_available;

   while (true) {
       // 检测遥控器是否离线，或者遥控器是否在安全模式下
       is_dbus_offline = (!dbus->IsOnline()) || dbus->swr == remote::DOWN;
#ifdef HAS_REFEREE
       // Kill Detection
       is_robot_dead = referee->game_robot_status.remain_HP == 0;
       is_shoot_available =
           referee->bullet_remaining.bullet_remaining_num_17mm > 0 && ahrs->IsCailbrated();
#else
       is_robot_dead = false;
       is_shoot_available = true;
#endif
       if (is_dbus_offline || is_robot_dead) {
           if (!is_killed) {
               // 如果遥控器离线或者机器人死亡，则进入安全模式
               last_remote_mode = remote_mode;
               remote_mode = REMOTE_MODE_KILL;
               // last_shoot_flywheel_mode = shoot_flywheel_mode;
               // shoot_flywheel_mode = SHOOT_FRIC_MODE_STOP;
               is_killed = true;
           }
       } else {
           if (is_killed) {
               // 如果遥控器重新连接或者机器人复活，则恢复上一次的遥控模式
               // shoot_flywheel_mode = last_shoot_flywheel_mode;
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


       // N键重新初始化
       if (dbus->IsOnline()) {
           keyboard_N_edge->input(dbus->keyboard.bit.C);
       } else {
           keyboard_N_edge->input(refereerc->remote_control.keyboard.bit.C);
       }

       if (keyboard_N_edge->posEdge()) {
           init_ui();
       }


#ifdef DBUS_MODE
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
       mouse_left_edge->input(mouse.l);
       mouse_right_edge->input(mouse.r);
       keyboard_shift_edge->input(keyboard.bit.SHIFT);
       keyboard_ctrl_edge->input(keyboard.bit.CTRL);
       keyboard_X_edge->input(dbus->keyboard.bit.X);

       keyboard_V_edge->input(keyboard.bit.V);
       keyboard_R_edge->input(keyboard.bit.R);
       keyboard_F_edge->input(keyboard.bit.F);

       uint16_t remain_HP = referee->game_robot_status.remain_HP;

       if (remain_HP == 0 && robot_iskill == false) {
           last_remote_mode = remote_mode;
           remote_mode = REMOTE_MODE_KILL;
           robot_iskill = true;
       }

       if (remain_HP != 0 && robot_iskill == true) {
           remote_mode = last_remote_mode;
           shoot_load_mode = SHOOT_MODE_STOP;
           robot_iskill = false;
       }

       if (keyboard_F_edge->posEdge()) {
           osDelay(REMOTE_MODE_OS_DELAY);
           remote_mode = REMOTE_MODE_FOLLOW;
       }

       if (keyboard_R_edge->posEdge()) {
           osDelay(REMOTE_MODE_OS_DELAY);
           remote_mode = REMOTE_MODE_ADVANCED;
       }

       if (keyboard_V_edge->posEdge()) {
           osDelay(REMOTE_MODE_OS_DELAY);
           remote_mode = REMOTE_MODE_SPIN;
       }




       // remote mode switch
       static BoolEdgeDetector* mode_switch_edge = new BoolEdgeDetector(false);
       mode_switch_edge->input(state_r == remote::UP);

       static BoolEdgeDetector* keyboard_G_edge = new BoolEdgeDetector(false);
       keyboard_G_edge->input(keyboard.bit.G);

       if (mode_switch_edge->posEdge() || keyboard_shift_edge->posEdge()) {
           RemoteMode next_mode = (RemoteMode)(remote_mode + 1);
           if ((int8_t)next_mode > (int8_t)remote_mode_max) {
               next_mode = (RemoteMode)remote_mode_min;
           }
           remote_mode = next_mode;
           if (remote_mode == REMOTE_MODE_AUTOAIM) {
               is_autoaim = true;
           } else {
               is_autoaim = false;
           }
       }

       // G键切换自瞄
       if (keyboard_G_edge->posEdge()) {
           is_autoaim = !is_autoaim;
           if (is_autoaim == false) {
               // gimbal->TargetAbs(INS_Angle.pitch, INS_Angle.yaw);
               shoot_load_mode = SHOOT_MODE_STOP;
           }
       }


       static BoolEdgeDetector* Magazine_switch_edge = new BoolEdgeDetector(false);
       Magazine_switch_edge->input(state_l == remote::UP);
       keyboard_B_edge->input(keyboard.bit.B);
       if (keyboard_B_edge->posEdge() || (shoot_load_mode == SHOOT_MODE_STOP && Magazine_switch_edge->posEdge())) {
           if (magazine_Mode==Magazine_MODE_ON) {
               magazine_Mode = Magazine_MODE_OFF;
           }else if (magazine_Mode == Magazine_MODE_OFF) {
               magazine_Mode = Magazine_MODE_ON;
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

       // 遥控器左拨杆,控制摩擦轮开关
       static BoolEdgeDetector* flywheel_switch_edge = new BoolEdgeDetector(false);
       flywheel_switch_edge->input(state_l == remote::DOWN);
       // 键盘Z键,控制摩擦轮开关
       keyboard_Z_edge->input(keyboard.bit.Z);
       if (flywheel_switch_edge->posEdge() || keyboard_Z_edge->posEdge()) {
           if (remote_mode != REMOTE_MODE_STOP){
               if (shoot_flywheel_mode == SHOOT_FRIC_MODE_STOP) {  // 原来停止则开始转
                   shoot_flywheel_mode = SHOOT_FRIC_MODE_PREPARING;
                   shoot_load_mode = SHOOT_MODE_IDLE;
               } else if (shoot_flywheel_mode == SHOOT_FRIC_MODE_PREPARING ||
                          shoot_flywheel_mode == SHOOT_FRIC_MODE_PREPARED) {  // 原来转则停止
                   shoot_flywheel_mode = SHOOT_FRIC_MODE_STOP;
                   shoot_load_mode = SHOOT_MODE_STOP;
               }
           }
       }

       if (!is_autoaim || !minipc->IsOnline()) {
           // 单发
           static BoolEdgeDetector* shoot_switch_edge = new BoolEdgeDetector(false);
           shoot_switch_edge->input(state_l == remote::UP);

           if (shoot_switch_edge->posEdge() || mouse_left_edge->posEdge()) {
               shoot_load_mode = SHOOT_MODE_SINGLE;
               shoot_burst_timestamp = 0;
           }

           // 连发
           if (mouse_right_edge->posEdge()) {
               shoot_burst_timestamp++;
           }
           static BoolEdgeDetector* shoot_burst_switch_edge = new BoolEdgeDetector(false);
           shoot_burst_switch_edge->input(shoot_burst_timestamp > 20 * REMOTE_OS_DELAY);
           if (mouse_right_edge->posEdge()) {
               shoot_load_mode = SHOOT_MODE_BURST;
           }

           if (keyboard_X_edge->posEdge()) {
               shoot_load_mode = SHOOT_MODE_UNLOAD;
           }

           // 不发射
           if (shoot_switch_edge->negEdge() || mouse_left_edge->negEdge() || mouse_right_edge->negEdge() || keyboard_X_edge->negEdge()) {
               shoot_load_mode = SHOOT_MODE_STOP;
               shoot_burst_timestamp = 0;
           }

       } else {
           // 自喵模式下只有连发
           if (minipc->IsOnline()) {
               if (minipc->target_angle.shoot_cmd != 0 && keyboard_G_edge->posEdge()) {
                   shoot_load_mode = SHOOT_MODE_BURST;
               } else {
                   shoot_load_mode = SHOOT_MODE_STOP;
                   shoot_burst_timestamp = 0;
               }
           }
       }
#endif

       osDelay(REMOTE_OS_DELAY);
   }
}
void init_remote() {
   init_controller();
}