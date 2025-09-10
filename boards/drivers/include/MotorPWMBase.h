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

#pragma once

#include "MotorBase.h"
#include "bsp_pwm.h"
#include "main.h"
namespace driver {
    /**
     * @brief PWM电机的标准类，用于一帧为20ms的通用PWM的电机与伺服
     */
    /**
     * @brief PWM motor base class, used for general PWM motor and servomotor with
     *       20ms frame
     */
    class MotorPWMBase : public MotorBase {
      public:
        /**
         * @brief 基础构造函数
         *
         * @param htim           HAL定时器句柄
         * @param channel        HAL定时器通道，取值范围为[0, 4)
         * @param clock_freq     定时器的时钟频率，单位为[Hz]
         * @param output_freq    期望的输出频率，单位为[Hz]
         * @param idle_throttle  空闲时的脉宽，单位为[us]
         *
         * @note M3508的idle_throttle约为1500，snail的idle_throttle约为1100
         */
        /**
         * @brief constructor
         *
         * @param htim           HAL timer handle
         * @param channel        HAL timer channel, from [0, 4)
         * @param clock_freq     clock frequency associated with the timer, in [Hz]
         * @param output_freq    desired output frequency, in [Hz]
         * @param idle_throttle  idling pulse width, in [us]
         *
         * @note M3508 have idle_throttle about 1500, snail have idle_throttle about
         * 1100
         */
        MotorPWMBase(TIM_HandleTypeDef* htim, uint8_t channel, uint32_t clock_freq,
                     uint32_t output_freq, uint32_t idle_throttle);

        /**
         * @brief 设置输出
         *
         * @param val 与idle_throttle的偏移量，单位为[us]
         */
        /**
         * @brief set and transmit output
         *
         * @param val offset value with respect to the idle throttle pulse width, in
         * [us]
         */
        virtual void SetOutput(int16_t val) override;

        void Enable();

        void Disable();

        bool isEnable() {
            return en_;
        }

      private:
        bsp::PWM pwm_;
        uint32_t idle_throttle_;
        bool en_;
    };

    /**
     * @brief DJI snail 2305电机的标准类
     */
    /**
     * @brief DJI snail 2305 motor class
     */
    class Motor2305 : public MotorPWMBase {
      public:
        /* override base implementation with max current protection */
        void SetOutput(int16_t val) override final;
    };
}  // namespace driver