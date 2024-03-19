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
#include "dbus_package.h"

namespace remote {

    typedef enum {
        UP = 1,
        DOWN = 2,
        MID = 3,
    } switch_t;

    /**
     * @brief DBUS 遥控器接收类
     * @note 用于DJI DR16接收机
     */
    /**
     * @brief DBUS remote receiver class
     * @note used for DJI DR16 receiver
     */
    class DBUS : public bsp::UART, public driver::ConnectionDriver {
      public:
        /**
         * @brief 构造函数
         * @note 和uart类似，dbus需要时间进行初始化
         *
         * @param huart uart实例
         */
        /**
         * @brief intialize DBUS the same way as a generic UART peripheral
         * @note like uart, dbus needs time to initialize
         *
         * @param huart uart instance
         */
        DBUS(UART_HandleTypeDef* huart);

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
        /**
         * @brief 获取右摇杆的x轴值
         */
        int16_t ch0;
        /**
         * @brief 获取右摇杆的y轴值
         */
        int16_t ch1;
        /**
         * @brief 获取左摇杆的x轴值
         */
        int16_t ch2;
        /**
         * @brief 获取左摇杆的y轴值
         */
        int16_t ch3;

        // left and right switch information
        /**
         * @brief 获取左拨杆的状态
         */
        switch_t swl;
        /**
         * @brief 获取右拨杆的状态
         */
        switch_t swr;
        // left track wheel
        /**
         * @brief 获取左波轮的x轴值
         */
        int16_t ch4;
        // mouse movement and button information
        /**
         * @brief 获取鼠标的数据，仅限于DT7连接上电脑之后
         */
        mouse_t mouse;
        // keyboard key information
        /**
         * @brief 获取键盘的数据，仅限于DT7连接上电脑之后
         */
        keyboard_t keyboard;

        static const int16_t ROCKER_MIN = -660;
        static const int16_t ROCKER_MAX = 660;

        uint32_t timestamp;
    };

} /* namespace remote */
