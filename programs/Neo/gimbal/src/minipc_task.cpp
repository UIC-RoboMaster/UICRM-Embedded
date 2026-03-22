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

// bsp::UART* minipc_uart = nullptr;
// communication::Host* minipc = nullptr;
bsp::VirtualUSB* minipc_usb = nullptr;
communication::HostUSB* minipc = nullptr;

bsp::Thread* minipc_thread = nullptr;

uint8_t start_time_stamp;
std::deque<uint8_t> time_queue;
constexpr uint8_t QSIZE = 50;

void limitSizePush(std::deque<uint8_t>& q, uint8_t val, uint8_t size) {
    if (q.size() > size)
        q.pop_front();
    q.push_back(val);
}

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
    minipc_usb = new bsp::VirtualUSB();
    //    minipc_uart = new bsp::UART(&BOARD_UART1);
    //    minipc_uart->SetBaudrate(921600);
    //    minipc_uart->SetupRx(300);
    //    minipc_uart->SetupTx(300);
    minipc = new communication::HostUSB(minipc_usb, 300, 300);
    //    minipc = new communication::Host(minipc_uart);

    // Start mini pc thread
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
            minipc->gimbal_current_status.current_imu_pitch = imu->INS_angle[1];
            minipc->gimbal_current_status.current_imu_yaw = imu->INS_angle[0];
            minipc->gimbal_current_status.current_imu_roll = imu->INS_angle[2];
            minipc->gimbal_current_status.robot_id = referee->game_robot_status.robot_id;
            minipc->gimbal_current_status.shooter_id = 0;
            start_time_stamp = 0;  // todo use RTC to recover the function since remotes' timestamp
                                   // no longer available
            limitSizePush(time_queue, start_time_stamp, QSIZE);
            minipc->gimbal_current_status.time_stamp = start_time_stamp;
            minipc->Transmit(communication::GIMBAL_CURRENT_STATUS);
        }
        if (i % 10 == 0) {
            minipc->chassis_current_status.chassis_enabled = 1;  // 暂时底盘默认为打开
            minipc->chassis_current_status.speed_x = chassis_vx;
            minipc->chassis_current_status.speed_y = chassis_vy;
            minipc->chassis_current_status.speed_turn = chassis_vt;
            minipc->Transmit(communication::CHASSIS_CURRENT_STATUS);
        }
        if (i % 500 == 0) {
            minipc->game_status.game_progress = referee->game_status.game_progress;
            minipc->game_status.game_type = referee->game_status.game_type;
            minipc->game_status.stage_remain_time = referee->game_status.stage_remain_time;
            minipc->game_status.SyncTimeStamp = referee->game_status.SyncTimeStamp;
            minipc->Transmit(communication::GAME_STATUS);
        }
        if (i % 999 == 0) {
            minipc->game_robot_status.max_HP = referee->game_robot_status.max_HP;
            minipc->game_robot_status.remain_HP = referee->game_robot_status.remain_HP;
            minipc->game_robot_status.chassis_power_limit = referee->game_robot_status.chassis_power_limit;
            minipc->game_robot_status.mains_power_chassis_output = referee->game_robot_status.mains_power_chassis_output;
            minipc->game_robot_status.mains_power_gimbal_output =  referee->game_robot_status.mains_power_gimbal_output;
            minipc->game_robot_status.mains_power_shooter_output = referee->game_robot_status.mains_power_shooter_output;
            minipc->game_robot_status.robot_id = referee->game_robot_status.robot_id;
            minipc->game_robot_status.robot_level = referee->game_robot_status.robot_level;
            minipc->game_robot_status.shooter_cooling_rate = referee->game_robot_status.shooter_cooling_rate;
            minipc->game_robot_status.shooter_heat_limit = referee->game_robot_status.shooter_heat_limit;
            minipc->Transmit(communication::GAME_ROBOT_STATUS);
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
