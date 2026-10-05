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
#include "protocols/dbus_receiver.h"

#include "bsp_os.h"

namespace remote {
    DbusReceiver::DbusReceiver(UART_HandleTypeDef* huart, uint32_t online_threshold_ms)
        : RemoteReceiver(online_threshold_ms), uart_(new bsp::UART(huart)), owns_uart_(true) {
        uart_->RegisterCallback(UartCallback, this);
        uart_->SetupRx(FRAME_SIZE + 1);
    }

    DbusReceiver::DbusReceiver(bsp::UART& uart, uint32_t online_threshold_ms)
        : RemoteReceiver(online_threshold_ms), uart_(&uart) {
        uart_->SetupRx(FRAME_SIZE + 1);
    }

    DbusReceiver::~DbusReceiver() {
        if (owns_uart_) delete uart_;
    }

    bool DbusReceiver::IsOnline() const {
        return RemoteReceiver::IsOnline(bsp::GetHighresTickMilliSec());
    }

    void DbusReceiver::UartCallback(void* context) {
        static_cast<DbusReceiver*>(context)->RxCompleteCallback();
    }

    void DbusReceiver::RxCompleteCallback() {
        if (!uart_) return;
        uint8_t* data = nullptr;
        const int32_t length = uart_->Read<true>(&data);
        UpdateData(data, length > 0 ? static_cast<size_t>(length) : 0,
                      bsp::GetHighresTickMilliSec());
    }

    void DbusReceiver::UpdateData(const uint8_t* data, size_t size, uint32_t now_ms) {
        // UART IDLE supplies a candidate frame boundary; only a full block is accepted.
        if (data != nullptr && size == FRAME_SIZE) {
            Decode(data, now_ms);
        }
    }

    bool DbusReceiver::SetChannelRange(uint16_t minimum, uint16_t maximum) {
        if (minimum < CHANNEL_MIN || minimum > maximum || maximum > CHANNEL_MAX) {
            return false;
        }
        channel_min_ = minimum;
        channel_max_ = maximum;
        return true;
    }

    bool DbusReceiver::GetState(DbusFrame& output) const {
        if (!HasState()) {
            return false;
        }
        output = state_;
        return true;
    }

    bool DbusReceiver::Decode(const uint8_t* data, uint32_t now_ms) {
        DbusFrame next;
        for (size_t i = 0; i < next.channels.size(); ++i) {
            // 处理 ch0-4
            const size_t offset = i < 4 ? 11 * i : 128;
            const uint16_t raw = detail::DecodeBits(data, offset, 11);

            // 检测范围 364 - 1024 - 1684
            if (raw < channel_min_ || raw > channel_max_) {
                return false;
            }
            // 原始值转通道值
            const int16_t value = static_cast<int16_t>(raw) - 1024;
            // 死区处理
            next.channels[i] = value >= -10 && value <= 10 ? 0 : value;
        }
        // 处理左右拨杆
        const auto right = detail::DecodeBits(data, 44, 2);
        const auto left = detail::DecodeBits(data, 46, 2);
        // 检测左右拨杆范围 + 鼠标左右键（data[12]/data[13])
        if (right == 0 || left == 0 || data[12] > 1 || data[13] > 1) {
            return false;
        }
        next.right_switch = static_cast<DbusSwitchPosition>(right);
        next.left_switch = static_cast<DbusSwitchPosition>(left);
        next.mouse.x = detail::DecodeInt16LE(data + 6);
        next.mouse.y = detail::DecodeInt16LE(data + 8);
        next.mouse.z = detail::DecodeInt16LE(data + 10);
        next.mouse.left = data[12] != 0;
        next.mouse.right = data[13] != 0;
        next.keyboard.code = detail::DecodeUint16LE(data + 14);
        state_ = next;
        CommitFrame(now_ms, false);
        return true;
    }
}  // namespace remote
