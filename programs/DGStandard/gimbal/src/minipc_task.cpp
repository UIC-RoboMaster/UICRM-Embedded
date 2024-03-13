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
        minipc->gimbal_current_status.current_imu_pitch = INS_Angle.pitch;
        minipc->gimbal_current_status.current_imu_yaw = INS_Angle.yaw;
        minipc->gimbal_current_status.current_imu_roll = INS_Angle.roll;
        minipc->Transmit(communication::ROBOT_POSITION);

        if (i == 1000) {
            // Secend event
            minipc->robot_status_upload.robot_id = referee->game_robot_status.robot_id;
            minipc->robot_status_upload.robot_level = referee->game_robot_status.robot_level;
            minipc->robot_status_upload.remain_HP = referee->game_robot_status.remain_HP;
            minipc->robot_status_upload.max_HP = referee->game_robot_status.max_HP;
            minipc->robot_status_upload.chassis_current_power =
                referee->power_heat_data.chassis_power;
            minipc->robot_status_upload.chassis_power_limit =
                referee->game_robot_status.chassis_power_limit;
            minipc->robot_status_upload.shooter_cooling_rate =
                referee->power_heat_data.shooter_id1_17mm_cooling_heat;
            minipc->robot_status_upload.shooter_heat_limit =
                referee->game_robot_status.shooter_heat_limit;
            minipc->Transmit(communication::ROBOT_STATUS_UPLOAD);
            i = 0;
        }
        osDelay(1);
    }
}
