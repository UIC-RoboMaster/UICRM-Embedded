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

#pragma once

#include "tim.h"

namespace bsp {

    /**
     * @brief PWM信号输出管理类
     * @details 用于输出PWM信号
     */
    /**
     * @brief PWM output manager
     * @details used to output PWM signal
     */
    class PWM {
      public:
        /**
         * @brief 构造函数
         *
         * @param htim         HAL定时器句柄
         * @param channel      定时器通道，可选[1,2,3,4]
         * @param clock_freq   定时器时钟频率，单位为Hz
         * @param output_freq  输出频率，单位为Hz
         * @param pulse_width  输出脉宽，单位为us
         */
        /**
         * @brief constructor for a pwm output manager
         *
         * @param htim         HAL timer handle
         * @param channel      channel associated with the timer, choose from
         * [1,2,3,4]
         * @param clock_freq   clock frequency associated with the timer, in [Hz]
         * @param output_freq  desired pwm output frequency, in [Hz]
         * @param pulse_width  desired pwm output pulse width, is [us]
         */
        PWM(TIM_HandleTypeDef* htim, uint8_t channel, uint32_t clock_freq, uint32_t output_freq,
            uint32_t pulse_width);

        /**
         * @brief 启动PWM信号输出
         */
        /**
         * @brief start pwm output signal generation
         */
        void Start();

        /**
         * @brief 停止PWM信号输出
         */
        /**
         * @brief stop pwm output signal generation
         */
        void Stop();

        /**
         * @brief 设置新的PWM输出频率
         *
         * @param output_freq   新的PWM输出频率，单位为Hz
         */
        /**
         * @brief set a new pwm output frequency
         *
         * @param output_freq   desired pwm output frequency, in [Hz]
         */
        void SetFrequency(uint32_t output_freq);

        /**
         * @brief 设置新的PWM输出脉宽
         *
         * @param pulse_width   新的PWM输出脉宽，单位为us
         */
        /**
         * @brief set a new pwm output pulse width
         *
         * @param pulse_width   desired pwm output pulse width, in [us]
         */
        void SetPulseWidth(uint32_t pulse_width);

      private:
        TIM_HandleTypeDef* htim_;
        uint32_t channel_;
        uint32_t clock_freq_;
        uint32_t output_freq_;
        uint32_t pulse_width_;
    };

} /* namespace bsp */
