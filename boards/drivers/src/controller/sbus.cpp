/*###########################################################
# Copyright (c) 2026-2027. BNBU/UIC RoboMaster             #
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

/**
 * @file sbus.cpp
 * @brief SBUS 字节流接收与 i6X 设备映射实现。
 * @details 函数顺序与 sbus.h
 * 一致；接口契约集中记录在头文件中。
 */
#include "sbus.h"

#include "bsp_os.h"

namespace controller {
    SBUS::SBUS(uint32_t online_threshold_ms, uint8_t footer)
        : ControllerReceiver(online_threshold_ms), footer_(footer) {
    }

    SBUS::SBUS(UART_HandleTypeDef* huart, uint32_t online_threshold_ms, uint8_t footer)
        : ControllerReceiver(online_threshold_ms), footer_(footer) {
        huart->Init.BaudRate = 100000;
        huart->Init.WordLength = UART_WORDLENGTH_9B;
        huart->Init.Parity = UART_PARITY_EVEN;
        huart->Init.StopBits = UART_STOPBITS_2;
        if (HAL_UART_Init(huart) != HAL_OK) {
            Error_Handler();
            return;
        }
        uart_ = new bsp::UART(huart);
        uart_->RegisterCallback(UartCallback, this);
        uart_->SetupRx(FRAME_SIZE + 1);
    }

    SBUS::~SBUS() {
        delete uart_;
    }

    bool SBUS::IsOnline() const {
        return ControllerReceiver::IsOnline(bsp::GetHighresTickMilliSec());
    }

    bool SBUS::GetRawState(SbusFrame& output) const {
        if (!HasState()) {
            return false;
        }
        output = state_;
        return true;
    }

    void SBUS::UpdateData(const uint8_t* data, size_t size, uint32_t now_ms) {
        if (data == nullptr) {
            return;
        }
        for (size_t i = 0; i < size; ++i) {
            if (used_ == 0 && data[i] != 0x0F) {
                continue;
            }
            buffer_[used_++] = data[i];
            if (used_ == FRAME_SIZE) {
                if (Decode(now_ms)) {
                    used_ = 0;
                } else {
                    detail::FindNextFrame(buffer_, used_, 0x0F);
                }
            }
        }
    }

    void SBUS::ResetStream() {
        used_ = 0;
    }

    void SBUS::SetFooter(uint8_t footer) {
        footer_ = footer;
        ResetStream();
    }

    void SBUS::UartCallback(void* context) {
        static_cast<SBUS*>(context)->RxCompleteCallback();
    }

    void SBUS::RxCompleteCallback() {
        if (!uart_)
            return;
        uint8_t* data = nullptr;
        const int32_t length = uart_->Read<true>(&data);
        if (length <= 0 || data == nullptr) {
            if (length != 0)
                ResetStream();
            return;
        }
        UpdateData(data, static_cast<size_t>(length), bsp::GetHighresTickMilliSec());
    }

    bool SBUS::Decode(uint32_t now_ms) {
        if (buffer_[0] != 0x0F || buffer_[24] != footer_) {
            return false;
        }
        SbusFrame next;
        for (size_t i = 0; i < next.channels.size(); ++i) {
            next.channels[i] = detail::DecodeBits(buffer_.data(), 8 + 11 * i, 11);
        }
        next.flags = buffer_[23];
        next.channel17 = (next.flags & 0x01) != 0;
        next.channel18 = (next.flags & 0x02) != 0;
        next.frame_lost = (next.flags & 0x04) != 0;
        next.failsafe = (next.flags & 0x08) != 0;
        state_ = next;
        CommitFrame(now_ms, next.failsafe);
        return true;
    }

    bool I6x::GetState(I6xState& output) const {
        SbusFrame raw;
        if (!GetRawState(raw)) {
            return false;
        }
        for (size_t i = 0; i < output.channels.size(); ++i) {
            output.channels[i] = MapChannel(raw.channels[i]);
        }
        output.sw_a = TwoPosition(raw.channels[6]);
        output.sw_b = TwoPosition(raw.channels[7]);
        output.sw_c = ThreePosition(raw.channels[8]);
        output.sw_d = TwoPosition(raw.channels[9]);
        output.frame_lost = raw.frame_lost;
        output.failsafe = raw.failsafe;
        return true;
    }

    int16_t I6x::MapChannel(uint16_t raw) {
        constexpr int32_t center = 1024;
        constexpr int32_t minimum = 240;
        constexpr int32_t maximum = 1807;
        constexpr int32_t range = 660;
        const int32_t offset = static_cast<int32_t>(raw) - center;
        if (offset >= -10 && offset <= 10) {
            return 0;
        }
        const bool negative = offset < 0;
        const int32_t span = negative ? center - minimum : maximum - center;
        const int32_t magnitude = negative ? -offset : offset;
        const int32_t scaled = magnitude >= span ? range : (magnitude * range + span / 2) / span;
        return static_cast<int16_t>(negative ? -scaled : scaled);
    }

    TwoPositionSwitch I6x::TwoPosition(uint16_t raw) {
        return raw >= 1024 ? TwoPositionSwitch::Up : TwoPositionSwitch::Down;
    }

    ThreePositionSwitch I6x::ThreePosition(uint16_t raw) {
        // Classify by the nearest calibrated position: 240, 1024, 1807.
        if (raw < (240 + 1024) / 2) {
            return ThreePositionSwitch::Down;
        }
        if (raw > (1024 + 1807) / 2) {
            return ThreePositionSwitch::Up;
        }
        return ThreePositionSwitch::Mid;
    }
}  // namespace controller
