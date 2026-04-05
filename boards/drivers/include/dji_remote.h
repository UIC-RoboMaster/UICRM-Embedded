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

#include "main.h"

namespace remote {
    typedef struct {
        int16_t x;
        int16_t y;
        int16_t z;
        uint8_t l;
        uint8_t r;
    } __packed mouse_t;

#define mouse_xy_max 32767.0

    typedef union {
        uint16_t code;
        struct {
            uint16_t W : 1;
            uint16_t S : 1;
            uint16_t A : 1;
            uint16_t D : 1;
            uint16_t SHIFT : 1;
            uint16_t CTRL : 1;
            uint16_t Q : 1;
            uint16_t E : 1;
            uint16_t R : 1;
            uint16_t F : 1;
            uint16_t G : 1;
            uint16_t Z : 1;
            uint16_t X : 1;
            uint16_t C : 1;
            uint16_t V : 1;
            uint16_t B : 1;
        } __packed bit;
    } __packed keyboard_t;

    typedef struct vt13_remote_t {
        uint64_t ch0 : 11;
        uint64_t ch1 : 11;
        uint64_t ch2 : 11;
        uint64_t ch3 : 11;
        enum vt13_mode_t : uint64_t {
            MODE_C = 0,
            MODE_N = 1,
            MODE_S = 2,
        } mode_sw : 2;
        uint64_t pause : 1;
        uint64_t swl : 1;
        uint64_t swr : 1;
        uint64_t ch4 : 11;
        uint64_t trigger : 1;
        constexpr static uint16_t ROCKER_MID = 1024;
        constexpr static uint16_t ROCKER_RANGE = 660;
    } __packed vt13_remote_t;

    typedef struct vt13_mouse_t {
        int16_t x;
        int16_t y;
        int16_t roll;
        struct {
            uint8_t l : 1;
            uint8_t reserved : 1; // 不知道为什么，VT13目前的包中间空了一位
            uint8_t r : 1;
            uint8_t mid : 1;
        };
        explicit operator mouse_t() const {
            mouse_t result;
            result.x = x;
            result.y = y;
            result.z = roll;
            result.l = l;
            result.r = r;
            return result;
        }
    } __packed vt13_mouse_t;

    typedef struct vt13_packet_t {
        constexpr static uint8_t SOF = 0xA9;
        constexpr static uint8_t CMD = 0x53;
        uint8_t sof;  // 0xA9
        uint8_t cmd;  // 0x53
        vt13_remote_t remote;
        vt13_mouse_t mouse;
        keyboard_t keyboard;
        uint16_t crc16;
    } __packed vt13_packet_t;
}  // namespace remote