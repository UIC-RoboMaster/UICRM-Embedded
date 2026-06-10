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

#include "tongji_vision.h"

namespace driver {
    static inline int16_t unpack_int16_be(const uint8_t* p) {
        return (int16_t)((uint16_t)p[0] << 8 | (uint16_t)p[1]);
    }

    /*
     * a. (uint16_t)p[0]：先把字节扩成 16 位无符号
     * b. << 8：高字节移到第 9-16 位
     * c. | p[1]：低字节填进第 1-8 位
     * d. (int16_t)：最后强转成有符号 int16，让负数（最高位是 1）被正确识别为负
     */

    TongjiVision::TongjiVision(bsp::CAN* can,
                               uint32_t send_canid,
                               uint32_t quat_canid,
                               uint32_t status_canid,
                               uint32_t online_timeout_ms)
        : can_(can),
          send_canid_(send_canid),
          quat_canid_(quat_canid),
          status_canid_(status_canid),
          online_timeout_ms_(online_timeout_ms) {
        can_->RegisterRxCallback(send_canid_, &TongjiVision::RxCallback, this);
    }

    bool TongjiVision::IsOnline() const {
        uint32_t now = osKernelGetTickCount(); // ms-tick
        return (now - last_rx_tick_) < online_timeout_ms_;
    }

    void TongjiVision::RxCallback(const uint8_t data[], void* args) {
        auto* self = static_cast<TongjiVision*>(args);
        Cmd c;
        c.control = (data[0] != 0);
        c.shoot = (data[1] != 0);
        c.yaw_rad = unpack_int16_be(&data[2]) / 1e4f;
        c.pitch_rad = unpack_int16_be(&data[4]) / 1e4f;
        c.horizon_m = unpack_int16_be(&data[6]) / 1e4f;
        self->cmd = c;
        self->last_rx_tick_ = osKernelGetTickCount();
    }

    bool TongjiVision::SendQuat(float qx, float qy, float qz, float qw) {
        int16_t ix = (int16_t)(qx * 1e4f);
        int16_t iy = (int16_t)(qy * 1e4f);
        int16_t iz = (int16_t)(qz * 1e4f);
        int16_t iw = (int16_t)(qw * 1e4f);
        uint8_t buf[8] = {
            (uint8_t)(ix >> 8), (uint8_t)ix,
            (uint8_t)(iy >> 8), (uint8_t)iy,
            (uint8_t)(iz >> 8), (uint8_t)iz,
            (uint8_t)(iw >> 8), (uint8_t)iw,
        };
        return can_->Transmit((uint16_t)quat_canid_, buf, 8) == 8;
    }


    bool TongjiVision::SendStatus(float bullet_speed_mps,
                                  uint8_t mode,
                                  uint8_t shoot_mode,
                                  float ft_angle_rad) {
        int16_t ibs = (int16_t)(bullet_speed_mps * 1e2f);
        int16_t ift = (int16_t)(ft_angle_rad * 1e4f);
        uint8_t buf[8] = {
            (uint8_t)(ibs >> 8), (uint8_t)ibs,
            mode,
            shoot_mode,
            (uint8_t)(ift >> 8), (uint8_t)ift,
            0, 0,
        };
        return can_->Transmit((uint16_t)status_canid_, buf, 8) == 8;
    }
}