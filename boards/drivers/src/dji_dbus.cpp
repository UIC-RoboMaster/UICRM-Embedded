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

#include "dji_dbus.h"

#include <cmath>
#include <cstring>

#include "bsp_error_handler.h"

/* rocker range and deadzones */
#define RC_ROCKER_MID 1024
#define RC_ROCKER_ZERO_DRIFT 10  // Range of possible drift around initial position
// Range of possible drift around min or max position

namespace remote {

    // helper struct for decoding raw bytes
    typedef struct {
        /* rocker channel information */
        uint16_t ch0 : 11;  // S1*             *S2
        uint16_t ch1 : 11;  //   C3-^       ^-C1
        uint16_t ch2 : 11;  // C2-<   >+ -<   >+C0
        uint16_t ch3 : 11;  //     +v       v+
        /* left and right switch information */
        uint8_t swr : 2;
        uint8_t swl : 2;
        /* mouse movement and button information */
        remote::mouse_t mouse;
        /* keyboard key information */
        remote::keyboard_t keyboard;
        uint16_t ch4 : 11;
    } __packed dbus_t;

    DBUS::DBUS(UART_HandleTypeDef* huart) : bsp::UART(huart) {
        ch0 = ch1 = ch2 = ch3 = ch4 = 0;
        swl = swr = DOWN;
        mouse = {};
        keyboard = {};
        SetupRx(sizeof(dbus_t) + 1);
    }

    void DBUS::RxCompleteCallback() {
        uint8_t* data;
        // data frame misalignment
        if (this->Read<true>(&data) != sizeof(dbus_t)) {
            ++rejected_frames;
            return;
        }

        // re-interpret the data buffer and decode into class properties
        dbus_t* repr = reinterpret_cast<dbus_t*>(data);
        // 拒绝明显错位的数据；四个摇杆通道允许少量端点误差。
        const auto valid_rocker = [](uint16_t value) {
            return value >= RC_ROCKER_MID - 700 && value <= RC_ROCKER_MID + 700;
        };
        if (!valid_rocker(repr->ch0) || !valid_rocker(repr->ch1) ||
            !valid_rocker(repr->ch2) || !valid_rocker(repr->ch3) ||
            repr->swl < UP || repr->swl > MID || repr->swr < UP || repr->swr > MID) {
            ++rejected_frames;
            return;
        }
        this->ch0 = repr->ch0 - RC_ROCKER_MID;
        this->ch1 = repr->ch1 - RC_ROCKER_MID;
        this->ch2 = repr->ch2 - RC_ROCKER_MID;
        this->ch3 = repr->ch3 - RC_ROCKER_MID;
        this->ch4 = repr->ch4 - RC_ROCKER_MID;
        this->ch0 = abs(this->ch0) <= RC_ROCKER_ZERO_DRIFT ? 0 : this->ch0;
        this->ch1 = abs(this->ch1) <= RC_ROCKER_ZERO_DRIFT ? 0 : this->ch1;
        this->ch2 = abs(this->ch2) <= RC_ROCKER_ZERO_DRIFT ? 0 : this->ch2;
        this->ch3 = abs(this->ch3) <= RC_ROCKER_ZERO_DRIFT ? 0 : this->ch3;
        this->ch4 = abs(this->ch4) <= RC_ROCKER_ZERO_DRIFT ? 0 : this->ch4;
        this->swl = static_cast<switch_t>(repr->swl);
        this->swr = static_cast<switch_t>(repr->swr);

        memcpy(&this->mouse, &repr->mouse, sizeof(mouse_t));
        memcpy(&this->keyboard, &repr->keyboard, sizeof(keyboard_t));
        ++accepted_frames;
        Heartbeat();
    }

} /* namespace remote */
