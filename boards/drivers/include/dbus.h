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

#include "bsp_uart.h"
#include "dbus_package.h"

namespace remote {

    typedef enum {
        UP = 1,
        DOWN = 2,
        MID = 3,
    } switch_t;

    class DBUS : public bsp::UART {
      public:
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
        int16_t ch0;  // S1*             *S2
        int16_t ch1;  //   C3-^       ^-C1
        int16_t ch2;  // C2-<   >+ -<   >+C0
        int16_t ch3;  //     +v       v+
        // left and right switch information
        switch_t swl;
        switch_t swr;
        // left track wheel
        int16_t ch4;
        // mouse movement and button information
        mouse_t mouse;
        // keyboard key information
        keyboard_t keyboard;
        // timestamp of the update interrupt
        uint32_t timestamp;

        volatile bool connection_flag_ = false;

        static const int16_t ROCKER_MIN = -660;
        static const int16_t ROCKER_MAX = 660;
    };

} /* namespace remote */
