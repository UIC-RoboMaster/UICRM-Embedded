// Copyright (c) 2026. BNU-HKBU UIC RoboMaster
// SPDX-License-Identifier: GPL-3.0-or-later
#include "protocols/sbus_receiver.h"

#include "bsp_os.h"

namespace remote {
    SbusReceiver::SbusReceiver(uint32_t online_threshold_ms, uint8_t footer)
        : RemoteReceiver(online_threshold_ms), footer_(footer) {
    }

    SbusReceiver::SbusReceiver(UART_HandleTypeDef* huart, uint32_t online_threshold_ms, uint8_t footer)
        : RemoteReceiver(online_threshold_ms), uart_(new bsp::UART(huart)), owns_uart_(true), footer_(footer) {
        uart_->RegisterCallback(UartCallback, this);
        uart_->SetupRx(FRAME_SIZE + 1);
    }

    SbusReceiver::SbusReceiver(bsp::UART& uart, uint32_t online_threshold_ms, uint8_t footer)
        : RemoteReceiver(online_threshold_ms), uart_(&uart), footer_(footer) {
        uart_->SetupRx(FRAME_SIZE + 1);
    }

    SbusReceiver::~SbusReceiver() {
        if (owns_uart_) delete uart_;
    }

    bool SbusReceiver::IsOnline() const {
        return RemoteReceiver::IsOnline(bsp::GetHighresTickMilliSec());
    }

    void SbusReceiver::UartCallback(void* context) {
        static_cast<SbusReceiver*>(context)->RxCompleteCallback();
    }

    void SbusReceiver::RxCompleteCallback() {
        if (!uart_) return;
        uint8_t* data = nullptr;
        const int32_t length = uart_->Read<true>(&data);
        if (length <= 0 || data == nullptr) {
            if (length != 0) ResetStream();
            return;
        }
        UpdateData(data, static_cast<size_t>(length), bsp::GetHighresTickMilliSec());
    }

    void SbusReceiver::UpdateData(const uint8_t* data, size_t size, uint32_t now_ms) {
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

    void SbusReceiver::ResetStream() {
        used_ = 0;
    }

    void SbusReceiver::SetFooter(uint8_t footer) {
        footer_ = footer;
        ResetStream();
    }

    bool SbusReceiver::GetState(SbusFrame& output) const {
        if (!HasState()) {
            return false;
        }
        output = state_;
        return true;
    }

    bool SbusReceiver::Decode(uint32_t now_ms) {
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
}  // namespace remote
