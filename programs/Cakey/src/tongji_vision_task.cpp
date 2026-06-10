// Copyright (c) 2026. BNU-HKBU UIC RoboMaster
//
// This program is free software: you can redistribute it
// and/or modify it under the terms of the GNU General
// Public License as published by the Free Software
// Foundation, either version 3 of the License, or (at
// your option) any later version.
//
// This program is distributed in the hope that it will be
// useful, but WITHOUT ANY WARRANTY; without even
// the implied warranty of MERCHANTABILITY or FITNESS
// FOR A PARTICULAR PURPOSE.  See the GNU General
// Public License for more details.
//
// You should have received a copy of the GNU General
// Public License along with this program.  If not, see
// <https://www.gnu.org/licenses/>.

//
// Created by Sarzn on 2026/6/10.
//


#include "tongji_vision_task.h"

#include "imu_task.h"
#include "public_port.h"
#include "referee_task.h"
#include "remote_task.h"

driver::TongjiVision* tongji_vision = nullptr;
osThreadId_t tongjiVisionTaskHandle;

const osThreadAttr_t tongjiVisionTaskAttribute = {
    .name = "TongjiVisionTask",
    .attr_bits = osThreadDetached,
    .cb_mem = nullptr,
    .cb_size = 0,
    .stack_mem = nullptr,
    .stack_size = 256 * 4,
    .priority = (osPriority_t)osPriorityHigh,
    .tz_module = 0,
    .reserved = 0,
};

void init_tongji_vision() {
    // 小电脑接到 CAN2, 如果改成 CAN1, 改这就行
    tongji_vision = new driver::TongjiVision(can2);
}

static uint8_t map_remote_mode_to_tongji(uint8_t remote_mode_value) {
    // remote_task.h: 1=FOLLOW, 2=SPIN, 3=ADVANCED, 4=AUTOAIM
    if (remote_mode_value == REMOTE_MODE_AUTOAIM) {
        return driver::TongjiVision::MODE_AUTO_AIM;
    }
    return driver::TongjiVision::MODE_IDLE;
    // 未来打能量机关时, 加 small_buff / big_buff 分支....
}

void tongjiVisionTask(void* arg) {
    UNUSED(arg);
    uint32_t i = 0;
    while (true) {
        // 500 Hz 发四元数
        // AHRS::GetQuat() 返回的顺序是 [w, x, y, z], 发送顺序 x, y, z, w
        if (ahrs != nullptr && ahrs->IsCailbrated()) {
            const float* q = ahrs->GetQuat();
            float qw = q[0];
            float qx = q[1];
            float qy = q[2];
            float qz = q[3];
            tongji_vision->SendQuat(qx, qy, qz, qw);
        }

        // 100 Hz 发弹速 + mode
        if (i % 5 == 0) {
            float bullet_speed = 0.0f;
            if (referee != nullptr) {
                bullet_speed = referee->shoot_data.bullet_speed;
            }
            uint8_t mode = map_remote_mode_to_tongji(remote_mode);
            tongji_vision->SendStatus(bullet_speed, mode, /*shoot_mode=*/0, /*ft_angle=*/0.0f);
        }

        i++;
        osDelay(2);
    }
}