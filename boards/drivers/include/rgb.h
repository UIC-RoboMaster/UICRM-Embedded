/*###########################################################
 # Copyright (c) 2023. BNU-HKBU UIC RoboMaster              #
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

#include "bsp_pwm.h"

namespace display {

    const uint32_t color_red = 0xFFFF0000;
    const uint32_t color_green = 0xFF00FF00;
    const uint32_t color_blue = 0xFF0000FF;
    const uint32_t color_yellow = 0xFFFFFF00;
    const uint32_t color_cyan = 0xFF00FFFF;
    const uint32_t color_magenta = 0xFFFF00FF;

    /**
     * @brief RGB LED
     *
     * @note 3条PWM通道，使用同一个定时器
     */
     /**
      * @brief RGB LED
      *
      * @note 3 PWM channels, using the same timer
      */
    class RGB {
      public:
        /**
         * @brief 构造函数
         * @param htim 定时器句柄
         * @param channelR 红色通道的PWM通道
         * @param channelG 绿色通道的PWM通道
         * @param channelB 蓝色通道的PWM通道
         * @param clock_freq 定时器时钟频率
         */
        /**
         * @brief Construct a new RGB object
         * @param htim timer handle
         * @param channelR the pwm channel for red
         * @param channelG the pwm channel for green
         * @param channelB the pwm channel for blue
         * @param clock_freq timer clock frequency
         */
        RGB(TIM_HandleTypeDef* htim, uint8_t channelR, uint8_t channelG, uint8_t channelB,
            uint32_t clock_freq);
        /**
         * @brief 显示一个颜色
         * @param aRGB
         * @note aRGB的格式为0xAARRGGBB
         * @note AA为透明度，RR为红色，GG为绿色，BB为蓝色
         * @note 透明度为0时，显示为黑色
         * @note 透明度为255时，显示为纯色
         * @note 透明度为1-254时，显示为混合色
         */
        /**
         * @brief Display a color
         * @param aRGB
         * @note aRGB format: 0xAARRGGBB
         * @note AA is alpha, RR is red, GG is green, BB is blue
         * @note when alpha is 0, display black
         * @note when alpha is 255, display pure color
         * @note when alpha is 1-254, display mixed color
         * @note alpha is the transparency
         */
        void Display(uint32_t aRGB);
        void Stop();

      private:
        bsp::PWM R_, G_, B_;
    };

}  // namespace display
