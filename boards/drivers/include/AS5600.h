// Copyright (c) 2025. BNU-HKBU UIC RoboMaster
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
// Created by RinChord on 2025/10/21.
//

#pragma once

#include "bsp_i2c.h"
#include "i2c.h"
#include "bsp_gpio.h"

// AS5600默认设备地址
// #define AS5600_ADDR 0x36
#define AS5600_WRITE_CMD 0x00;
#define AS5600_READ_CMD 0x01
namespace imu
{
    typedef struct {
        // bsp::SPIMaster* spi;
        bsp::I2C* hi2c;
        uint16_t i2c_addr;
    } as5600_init_t;

    class AS5600
    {
    private:
        volatile uint16_t angle_ = -1;                      // 当前编码器所在角度，单位为[rad]
        volatile uint16_t rawangle_ = -1;                      // 当前编码器所在角度，单位为[rad]
        // volatile float speed_;                      // 当前编码器速度，单位为[rad/s]

        // volatile float output_shaft_angle_;         // 编码器的累计角度，单位为[rad]
        // volatile float output_shaft_speed_;         // 编码器的速度，单位为[rad/s]

        volatile uint16_t power_on_angle_ = 0;         /* 上电时的编码器角度，单位为[rad] */
        volatile uint16_t relative_angle_ = 0;         /* 编码器相对于开机角度的角度，单位为[rad] */
        // volatile float cumulated_turns_ = 0;        /* 编码器累计圈数，按照2*PI/ratio加减，累积到2*PI清零 */
        // volatile float output_cumulated_turns_ = 0; /* 输出轴累计圈数，按照2*PI加减，单位为[rad]*/
        // volatile float output_relative_angle_ = 0;  /* 输出轴在这一圈中的角度，单位为[rad]，范围为[0, 2PI] */

    public:
        explicit AS5600(as5600_init_t init);

        uint16_t GetAngle();
        uint16_t GetRawAngle();

        /**
         * @brief 读取原始角度数据
         */
        void ReadAngle();

        /**
         * @brief 读取缩放后角度数据（精度提高，需要配置角度限幅)
         */
        void ReadRawAngle();

        /**
         * @brief 检查设备是否就绪
         * @return bool true if device is ready, otherwise false
         */
        bool IsReady();

        /**
         * @brief 初始化AS5600设备
         */
        void Init();

        /**
         * @brief 复位编码器的角度计数
         */
        void Cailbrate();

        /**
         * @brief 更新数据
         */
        void Update();

        /**
         * @brief (生产中不使用) 烧录配置文件，仅可烧录三次
         */
        void Burn();

        /**
         * @brief 读取写进去的配置
         */
        void ReadConfig();

        void ReadByte();

        void WriteByte();

        bsp::I2C* i2c_;
        uint16_t i2c_addr_;

    };
} // AS5600

