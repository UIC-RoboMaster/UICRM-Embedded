/*###########################################################
 # Copyright (c) 2026. BNBU-HKBU UIC RoboMaster             #
 #                                                          #
 # This program is free software: you can redistribute it   #
 # and/or modify it under the terms of the GNU General      #
 # Public License as published by the Free Software         #
 # Foundation, either version 3 of the License, or (at      #
 # your option) any later version.                          #
 #                                                          #
 # This program is distributed in the hope that it will be  #
 # useful, but WITHOUT ANY WARRANTY; without even           #
 # the implied warranty of MERCHANTABILITY or FITNESS FOR   #
 # A PARTICULAR PURPOSE.  See the GNU General Public License#
 # for more details.                                        #
 #                                                          #
 # You should have received a copy of the GNU General       #
 # Public License along with this program.  If not, see     #
 # <https://www.gnu.org/licenses/>.                         #
 ###########################################################*/

/*
 * 上位机链路联调（UART）：USART6 / 921600 8N1。
 *
 * 上行：0x0500/0x0501/0x0502/0x0503/0x0504 全部字段填 1，
 *       只有 uptime_ms 用真实时间、shot_count 回填下行帧计数，
 *       上位机一眼能认出这是测试帧，也能从 shot_count 判断回程是否通。
 * 下行：0x0400/0x0401 收到就记录，并在 RTT 打印最近一帧。
 * 日志：RTT 通道 0，每 500 ms 一行。
 */

#include "main.h"

#include "bsp_print.h"
#include "bsp_uart.h"
#include "cmsis_os.h"
#include "host.h"

namespace {
    constexpr uint32_t LINK_BAUDRATE = 921600;
    constexpr uint32_t BUFFER_SIZE = 1024;
    constexpr uint32_t PRINT_PERIOD_MS = 500;

    host::ByteLink* link = nullptr;
    host::Host* comm = nullptr;

    /** @brief 所有字段填 1，让上位机一眼看出是联调帧。 */
    void FillTestPayload() {
        comm->version.host_version = 1;
        comm->version.referee_season = 1;
        comm->version.robot_type = 1;
        comm->version.chassis_type = 1;
        comm->version.gimbal_type = 1;

        comm->robot_status.enable = 1;
        comm->robot_status.controller_online = 1;
        comm->robot_status.robot_mode = 1;
        comm->robot_status.fric_mode = 1;
        comm->robot_status.shoot_mode = 1;
        comm->robot_status.module_online = 1;

        comm->chassis_status.chassis_enabled = 1;
        comm->chassis_status.speed_x = 1.0f;
        comm->chassis_status.speed_y = 1.0f;
        comm->chassis_status.speed_turn = 1.0f;
        comm->chassis_status.feedback_angle = 1.0f;

        comm->gimbal_status.gimbal_enable = 1;
        comm->gimbal_status.autoaim_enabled = 1;
        comm->gimbal_status.pitch_angle = 1.0f;
        comm->gimbal_status.yaw_angle = 1.0f;
        comm->gimbal_status.roll_angle = 1.0f;
        comm->gimbal_status.upper_yaw_angle = 1.0f;
        comm->gimbal_status.fold_yaw_angle = 1.0f;

        comm->shoot_status.shoot_enable = 1;
        comm->shoot_status.fric_state = 1;
        comm->shoot_status.load_state = 1;
    }

    void PrintStatus() {
        const host::Host::Stats& s = comm->stats();
        print(
            "tx=%luB/%luf rx=%lu/%lu drop=%lu retry=%lu\r\n",
            static_cast<unsigned long>(s.tx_bytes),
            static_cast<unsigned long>(s.tx_frames),
            static_cast<unsigned long>(s.rx_frames),
            static_cast<unsigned long>(s.rx_rejected),
            static_cast<unsigned long>(s.forward_dropped),
            static_cast<unsigned long>(s.tx_retry)
        );
        print(
            "minipc=%d aim=%d nav=%d\r\n",
            comm->IsOnline() ? 1 : 0,
            comm->AimValid() ? 1 : 0,
            comm->NavigationValid() ? 1 : 0
        );
        print("aim %.3f %.3f shoot=%u\r\n", comm->aim.pitch, comm->aim.yaw, comm->aim.shoot);
        print(
            "nav %.3f %.3f %.3f ang=%.3f\r\n",
            comm->navigation.vx,
            comm->navigation.vy,
            comm->navigation.wz,
            comm->navigation.target_angle
        );
    }
}  // namespace

void RM_RTOS_Init(void) {
    // 日志走 RTT，串口整个让给上位机。
    print_use_rtt();

    auto* uart = new bsp::UART(&huart6);
    uart->SetBaudrate(LINK_BAUDRATE);
    uart->SetupRx(BUFFER_SIZE);
    uart->SetupTx(BUFFER_SIZE);
    link = new host::UartLink(uart);

    comm = new host::Host(*link);
    FillTestPayload();
}

void RM_RTOS_Threads_Init(void) {
}

void RM_RTOS_Default_Task(const void* arg) {
    UNUSED(arg);
    print("Host link test: USART6 921600 8N1, log on RTT ch0\r\n");
    print("TX all fields = 1; shot_count mirrors rx frames\r\n");

    uint32_t last_print = HAL_GetTick();
    while (true) {
        const uint32_t now = HAL_GetTick();
        // 两个例外：时间戳必须递增，shot_count 回填下行计数。
        comm->version.uptime_ms = now;
        comm->shoot_status.shot_count = static_cast<uint16_t>(comm->stats().rx_frames);

        comm->Pump(now);

        if (now - last_print >= PRINT_PERIOD_MS) {
            last_print = now;
            PrintStatus();
        }
        osDelay(1);
    }
}
