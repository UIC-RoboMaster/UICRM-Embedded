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

#include "sbus.h"

#include <cmath>
#include <cstring>

#include "bsp_error_handler.h"
#include "utils.h"

/* rocker range and deadzones */
#define SBUS_RC_ROCKER_MID 1024
#define SBUS_RC_ROCKER_ZERO_DRIFT 10  // Range of possible drift around initial position
// Range of possible drift around min or max position

namespace remote {

    // helper struct for decoding raw bytes
    typedef struct {
        uint8_t start : 8;
        uint16_t ch1 : 11;
        uint16_t ch2 : 11;
        uint16_t ch3 : 11;
        uint16_t ch4 : 11;
        uint16_t ch5 : 11;
        uint16_t ch6 : 11;
        uint16_t ch7 : 11;
        uint16_t ch8 : 11;
        uint16_t ch9 : 11;
        uint16_t ch10 : 11;
        uint16_t ch11 : 11;
        uint16_t ch12 : 11;
        uint16_t ch13 : 11;
        uint16_t ch14 : 11;
        uint16_t ch15 : 11;
        uint16_t ch16 : 11;
        uint8_t flag : 8;
        uint8_t end : 8;
    } __packed sbus_t;

    SBUS::SBUS(UART_HandleTypeDef* huart) : bsp::UART(huart) {
        SetupRx(sizeof(sbus_t) + 1);
    }

    void SBUS::RxCompleteCallback() {
        connection_flag_ = true;

        uint8_t* data;
        // data frame misalignment
        if (this->Read<true>(&data) != sizeof(sbus_t))
            return;

        // re-interpret the data buffer and decode into class properties
        sbus_t* repr = reinterpret_cast<sbus_t*>(data);
        this->ch1 = clip<int16_t>(repr->ch1 - SBUS_RC_ROCKER_MID, -660, 660);
        this->ch2 = clip<int16_t>(repr->ch2 - SBUS_RC_ROCKER_MID, -660, 660);
        this->ch3 = clip<int16_t>(repr->ch3 - SBUS_RC_ROCKER_MID, -660, 660);
        this->ch4 = clip<int16_t>(repr->ch4 - SBUS_RC_ROCKER_MID, -660, 660);
        this->ch5 = clip<int16_t>(repr->ch5 - SBUS_RC_ROCKER_MID, -660, 660);
        this->ch6 = clip<int16_t>(repr->ch6 - SBUS_RC_ROCKER_MID, -660, 660);
        this->ch7 = clip<int16_t>(repr->ch7 - SBUS_RC_ROCKER_MID, -660, 660);
        this->ch8 = clip<int16_t>(repr->ch8 - SBUS_RC_ROCKER_MID, -660, 660);
        this->ch9 = clip<int16_t>(repr->ch9 - SBUS_RC_ROCKER_MID, -660, 660);
        this->ch10 = clip<int16_t>(repr->ch10 - SBUS_RC_ROCKER_MID, -660, 660);
        this->ch11 = clip<int16_t>(repr->ch11 - SBUS_RC_ROCKER_MID, -660, 660);
        this->ch12 = clip<int16_t>(repr->ch12 - SBUS_RC_ROCKER_MID, -660, 660);
        this->ch13 = clip<int16_t>(repr->ch13 - SBUS_RC_ROCKER_MID, -660, 660);
        this->ch14 = clip<int16_t>(repr->ch14 - SBUS_RC_ROCKER_MID, -660, 660);
        this->ch15 = clip<int16_t>(repr->ch15 - SBUS_RC_ROCKER_MID, -660, 660);
        this->ch16 = clip<int16_t>(repr->ch16 - SBUS_RC_ROCKER_MID, -660, 660);
        this->ch1 = abs(this->ch1) <= SBUS_RC_ROCKER_ZERO_DRIFT ? 0 : this->ch1;
        this->ch2 = abs(this->ch2) <= SBUS_RC_ROCKER_ZERO_DRIFT ? 0 : this->ch2;
        this->ch3 = abs(this->ch3) <= SBUS_RC_ROCKER_ZERO_DRIFT ? 0 : this->ch3;
        this->ch4 = abs(this->ch4) <= SBUS_RC_ROCKER_ZERO_DRIFT ? 0 : this->ch4;
        this->ch5 = abs(this->ch5) <= SBUS_RC_ROCKER_ZERO_DRIFT ? 0 : this->ch5;
        this->ch6 = abs(this->ch6) <= SBUS_RC_ROCKER_ZERO_DRIFT ? 0 : this->ch6;
        this->ch7 = abs(this->ch7) <= SBUS_RC_ROCKER_ZERO_DRIFT ? 0 : this->ch7;
        this->ch8 = abs(this->ch8) <= SBUS_RC_ROCKER_ZERO_DRIFT ? 0 : this->ch8;
        this->ch9 = abs(this->ch9) <= SBUS_RC_ROCKER_ZERO_DRIFT ? 0 : this->ch9;
        this->ch10 = abs(this->ch10) <= SBUS_RC_ROCKER_ZERO_DRIFT ? 0 : this->ch10;
        this->ch11 = abs(this->ch11) <= SBUS_RC_ROCKER_ZERO_DRIFT ? 0 : this->ch11;
        this->ch12 = abs(this->ch12) <= SBUS_RC_ROCKER_ZERO_DRIFT ? 0 : this->ch12;
        this->ch13 = abs(this->ch13) <= SBUS_RC_ROCKER_ZERO_DRIFT ? 0 : this->ch13;
        this->ch14 = abs(this->ch14) <= SBUS_RC_ROCKER_ZERO_DRIFT ? 0 : this->ch14;
        this->ch15 = abs(this->ch15) <= SBUS_RC_ROCKER_ZERO_DRIFT ? 0 : this->ch15;
        this->ch16 = abs(this->ch16) <= SBUS_RC_ROCKER_ZERO_DRIFT ? 0 : this->ch16;
        this->flag = repr->flag;
        this->timestamp = HAL_GetTick();
    }

} /* namespace remote */
