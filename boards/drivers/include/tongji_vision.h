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
// Created by Sarzn on 2026/6/9.
//

#ifndef UICRM_TONGJI_VISION_H
#define UICRM_TONGJI_VISION_H

#pragma once
#include <cstdint>

#include "bsp_can.h"
#include "cmsis_os.h"

namespace driver {
    /**
     *@brief 同济 sp_vision_25 自瞄链路(CAN)的协议封装
     *协议 source: sp_vision_25/io/cboard.cpp
     *
     * - RX ID send_canid (default 0xff): {control, shoot, yaw*1e4, pitch*1e4, horizon_dist*1e4}
     * - TX ID quat_canid (default 0x100): {qx, qy, qz, qw} (*1e4)
     * - TX ID status_canid (default 0x101): {bullet_speed*1e2, mode, shoot_mode, ft*1e4}
     * 大端, 有符号 int16
     *
     */
    class TongjiVision {
    public:
        enum Mode : uint8_t {
            MODE_IDLE = 0,
            MODE_AUTO_AIM = 1,
            MODE_SMALL_BUFF = 2,
            MODE_BIG_BUFF = 3,
            MODE_OUTPOST = 4,
        };

        enum ShootMode : uint8_t {
            SHOOT_LEFT = 0,
            SHOOT_RIGHT = 1,
            SHOOT_BOTH = 2,
        };

        struct Cmd {
            bool control;    // 是否启动云台自瞄
            bool shoot;      // 是否要求开火
            float yaw_rad;   // 旋转弧度 - 绝对世界
            float pitch_rad; // pitch旋转弧度 - 绝对世界
            float horizon_m; // 水平距离 - 无人机需要
        };

        /**
         * @param can 接受/发送所要使用的 CAN 总线
         * @param send_canid 小电脑发给 MCU 的  命令 ID       - 默认 0xff
         * @param quat_canid MCU 发给小电脑的   四元数 ID     - 默认 0x100
         * @param status_canid MCU 发给小电脑的 弹速/模式 ID  - 默认 0x101
         * @param online_timeout_ms 多少 ms 没有接收到 0xff 算作掉线
         */

        TongjiVision(bsp::CAN* can,
                     uint32_t send_canid = 0xff,
                     uint32_t quat_canid = 0x100,
                     uint32_t status_canid = 0x101,
                     uint32_t online_timeout_ms = 200);

        Cmd cmd{}; // 值初始化
        bool IsOnline() const;

        // 发送四元数(x,y,z,w) 返回是否调用 CAN 成功
        bool SendQuat(float qx, float qy, float qz, float qw);


        /**
         * @param bullet_speed_mps 当前弹速 m/s - 裁判系统提供
         * @param mode             当前 robot mode - 见 Mode 枚举
         * @param shoot_mode       仅哨兵用，Cakey 传 0
         * @param ft_angle_rad     仅无人机用，Cakey 传 0
         */
        bool SendStatus(float bullet_speed_mps,
                        uint8_t mode,
                        uint8_t shoot_mode = 0,
                        float ft_angle_rad = 0.0f);

    private:
        static void RxCallback(const uint8_t data[], void* args); // CAN 中断回调函数

        bsp::CAN* can_;
        uint32_t send_canid_;
        uint32_t quat_canid_;
        uint32_t status_canid_;
        uint32_t online_timeout_ms_;
        uint32_t last_rx_tick_ = 0;
    };
}

#endif