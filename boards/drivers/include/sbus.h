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

#include "bsp_uart.h"
#include "connection_driver.h"

namespace remote {

    /**
     * @brief SBUS 遥控器接收类
     * @note 用于支持SBUS的接收机
     */
    /**
     * @brief DBUS remote receiver class
     * @note used for DJI DR16 receiver
     */
    class SBUS : public bsp::UART, public driver::ConnectionDriver {
      public:
        /**
         * @brief 构造函数
         * @note 和uart类似，sbus需要时间进行初始化
         *
         * @param huart uart实例
         */
        /**
         * @brief intialize SBUS the same way as a generic UART peripheral
         * @note like uart, sbus needs time to initialize
         *
         * @param huart uart instance
         */
        SBUS(UART_HandleTypeDef* huart);

        // Add custom rx data handler
        void RxCompleteCallback() override final;

        // rocker channel information
        /**
         * @note 遥控器的样式
         *
         * @note the style of the remote
         *
         * C4(
         * SWL*           *SWR
         *   C3-^       ^-C1
         * C2-<   >+ -<   >+C0
         *     +v       v+
         *
         */

        volatile int16_t ch1;
        volatile int16_t ch2;
        volatile int16_t ch3;
        volatile int16_t ch4;
        volatile int16_t ch5;
        volatile int16_t ch6;
        volatile int16_t ch7;
        volatile int16_t ch8;
        volatile int16_t ch9;
        volatile int16_t ch10;
        volatile int16_t ch11;
        volatile int16_t ch12;
        volatile int16_t ch13;
        volatile int16_t ch14;
        volatile int16_t ch15;
        volatile int16_t ch16;
        volatile uint8_t flag;

        // timestamp of the update interrupt
        /**
         * @brief 获取更新中断的时间戳
         */
        uint32_t timestamp;

        static const int16_t ROCKER_MIN = -1023;
        static const int16_t ROCKER_MAX = 1023;
    };

} /* namespace remote */
