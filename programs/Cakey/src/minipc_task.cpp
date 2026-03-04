/*###########################################################
 # Copyright (c) 2024. BNU-HKBU UIC RoboMaster              #
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

#include "minipc_task.h"

#include "bsp_thread.h"
#include "bsp_uart.h"
#include "chassis_task.h"
#include "gimbal_task.h"
#include "protocol.h"
#include "referee_task.h"

bsp::UART* minipc_uart = nullptr;
communication::Host* minipc = nullptr;

bsp::Thread* minipc_thread = nullptr;
const osThreadAttr_t minipc_thread_attr_ = {.name = "MiniPCTask",
                                            .attr_bits = osThreadDetached,
                                            .cb_mem = nullptr,
                                            .cb_size = 0,
                                            .stack_mem = nullptr,
                                            .stack_size = 256 * 4,
                                            .priority = (osPriority_t)osPriorityHigh,
                                            .tz_module = 0,
                                            .reserved = 0};

void minipc_task(void* args);

const bsp::thread_init_t thread_init = {
    .func = minipc_task,
    .args = nullptr,
    .attr = minipc_thread_attr_,
};

void init_minipc() {
    minipc_uart = new bsp::UART(&huart6);
    minipc_uart->SetBaudrate(921600);
    minipc_uart->SetupRx(300);
    minipc_uart->SetupTx(300);
    minipc = new communication::Host(minipc_uart);
    minipc_thread = new bsp::Thread(thread_init);
    minipc_thread->Start();
}

void minipc_task(void* args) {
    UNUSED(args);
    uint16_t i = 0;
    while (true) {
        i++;
        // microsecond
        if (i % 2 == 0) {
            minipc->gimbal_current_status.current_imu_pitch = INS_Angle.pitch;
            minipc->gimbal_current_status.current_imu_yaw = INS_Angle.yaw;
            minipc->gimbal_current_status.current_imu_roll = INS_Angle.roll;
            minipc->gimbal_current_status.robot_id = referee->game_robot_status.robot_id;
            minipc->gimbal_current_status.shooter_id = 0;
            minipc->Transmit(communication::GIMBAL_CURRENT_STATUS);
        }
        if (i % 10 == 0) {
            minipc->chassis_current_status.chassis_enabled = 1;  // 暂时底盘默认为打开
            minipc->chassis_current_status.speed_x = chassis_vx;
            minipc->chassis_current_status.speed_y = chassis_vy;
            minipc->chassis_current_status.speed_turn = chassis_vt;
            minipc->Transmit(communication::CHASSIS_CURRENT_STATUS);
        }

        if (i == 1000) {
            // Secend event
            minipc->robot_status_upload.robot_id = referee->game_robot_status.robot_id;
            minipc->robot_status_upload.vision_reset = 0;
            minipc->robot_status_upload.location_data[0] = 0;
            minipc->robot_status_upload.location_data[1] = 0;
            minipc->robot_status_upload.is_killed = (referee->game_robot_status.remain_HP == 0);
            minipc->robot_status_upload.is_killed |= (remote_mode == REMOTE_MODE_KILL);
            switch (remote_mode) {
                case REMOTE_MODE_FOLLOW:
                    minipc->robot_status_upload.robot_mode = 1;
                    break;
                case REMOTE_MODE_SPIN:
                    minipc->robot_status_upload.robot_mode = 2;
                    break;
                case REMOTE_MODE_ADVANCED:
                    minipc->robot_status_upload.robot_mode = 3;
                    break;
                default:
                    minipc->robot_status_upload.robot_mode = 0;
                    break;
            }
            switch (shoot_flywheel_mode) {
                case SHOOT_FRIC_MODE_PREPARED:
                    minipc->robot_status_upload.robot_fric_mode = 1;
                    break;
                default:
                    minipc->robot_status_upload.robot_fric_mode = 0;
                    break;
            }
            switch (shoot_load_mode) {
                case SHOOT_MODE_SINGLE:
                    // 单发状态未必会更新，并且未来可能会直接全部突突突取消单发模式
                    minipc->robot_status_upload.robot_shoot_mode = 1;
                    break;
                case SHOOT_MODE_BURST:
                    minipc->robot_status_upload.robot_shoot_mode = 2;
                    break;
                default:
                    minipc->robot_status_upload.robot_shoot_mode = 0;
                    break;
            }
            minipc->robot_status_upload.supercapacitor_enabled = 0;
            minipc->robot_status_upload.robot_module_online = 0xff;
            minipc->robot_status_upload.is_double_gimbal = 0;
            minipc->Transmit(communication::ROBOT_STATUS_UPLOAD);
            i = 0;
        }
        osDelay(1);
    }
}
