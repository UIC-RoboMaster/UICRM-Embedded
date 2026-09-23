/*###########################################################
 # Copyright (c) 2026-2027. BNU-HKBU UIC RoboMaster         #
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

#pragma once

#include "bsp_can.h"
#include "connection_driver.h"

namespace driver {

    /**
     * @brief AS5047P CAN 磁编码器模块的初始化结构体
     */
    struct as5047p_encoder_init_t {
        bsp::CAN* can;
        /* 模块发送 ID，为 0x400 + 模块 ID，模块 ID 存于模块自身的 EEPROM */
        uint16_t rx_id;
        /**
         * 安装零点 [rad]：被测轴处于机械零位时磁编读到的角度。
         * @warning 扣除发生在 reversed 取反之后，reversed 为 true 时本值须同步取负。
         */
        float offset = 0;
        /* 磁编正方向与被测轴正方向相反时置 true */
        bool reversed = false;
    };

    /**
     * @brief AS5047P CAN 磁编码器模块的接收驱动
     *
     * @details 模块以 1kHz 单向广播，标准帧 8 字节，全部为大端：
     *          | 字节 | 含义                             | 类型   |
     *          | 0-1  | 单圈角度，已由模块减去其自身零点 | uint16 |
     *          | 2-3  | 转速 [rpm]，模块内滑动平均       | int16  |
     *          | 4-7  | 累计计数，16384 计数/圈          | int32  |
     */
    class AS5047PEncoder : public ConnectionDriver {
      public:
        /* AS5047P 为 14 位单圈绝对编码器 */
        static constexpr uint16_t COUNTS_PER_REV = 16384;

        explicit AS5047PEncoder(const as5047p_encoder_init_t& init);

        /** @brief 单圈原始计数，不含 offset，范围 0 ~ 16383 */
        uint16_t GetRawCount() const;

        /** @brief 扣除安装零点后的单圈绝对角度 [rad]，范围 [0, 2π) */
        float GetAngle() const;

        /** @brief 扣除安装零点后的单圈绝对角度 [rad]，范围 [-π, π) */
        float GetAngleWrapped() const;

        /** @brief 角速度 [rad/s] */
        float GetOmega() const;

        /** @brief 转速 [rpm] */
        float GetRpm() const;

        /**
         * @brief 模块自上电以来的累计角度 [rad]
         * @warning 相对量，开机恒为 0，不含绝对位置信息
         */
        float GetCumulatedAngle() const;

        /** @brief 模块自上电以来的累计计数，同样是相对量 */
        int32_t GetRawCumulatedCount() const;

        void SetOffset(float offset);
        float GetOffset() const;

        /** @brief 把被测轴的当前位置标定为零点，结果不持久化 */
        void AlignHere();

      private:
        static void CallbackWrapper(const uint8_t data[], void* args);
        void UpdateData(const uint8_t data[]);
        float RawToRad() const;

        bsp::CAN* can_;
        uint16_t rx_id_;
        float offset_;
        bool reversed_;

        volatile uint16_t raw_count_ = 0;
        volatile int16_t raw_rpm_ = 0;
        volatile int32_t raw_cumulated_count_ = 0;
    };

}  // namespace driver
