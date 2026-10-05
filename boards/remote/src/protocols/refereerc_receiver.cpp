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
#include "protocols/refereerc_receiver.h"

#include "crc_check.h"

namespace remote {
    void RefereeRcReceiver::UpdateData(const uint8_t* data, size_t size, uint32_t now_ms) {
        if (data == nullptr) {
            return;
        }
        for (size_t i = 0; i < size; ++i) {
            if (used_ == 0 && data[i] != SOF) {
                continue;
            }
            buffer_[used_++] = data[i];
            if (used_ == FRAME_SIZE) {
                if (Decode(now_ms)) {
                    used_ = 0;
                } else {
                    detail::FindNextFrame(buffer_, used_, SOF);
                }
            }
        }
    }

    void RefereeRcReceiver::ResetStream() {
        used_ = 0;
    }

    bool RefereeRcReceiver::GetState(Vt13Frame& output) const {
        if (!HasState()) {
            return false;
        }
        output = state_;
        return true;
    }

    bool RefereeRcReceiver::Decode(uint32_t now_ms) {
        if (buffer_[0] != SOF || buffer_[1] != CMD || !verify_crc16_check_sum(buffer_.data(), FRAME_SIZE)) {
            return false;
        }
        Vt13Frame next;
        for (size_t i = 0; i < next.channels.size(); ++i) {
            const size_t offset = i < 4 ? 16 + 11 * i : 65;
            const uint16_t raw = detail::DecodeBits(buffer_.data(), offset, 11);
            if (raw < CHANNEL_MIN || raw > CHANNEL_MAX) {
                return false;
            }
            const int16_t value = static_cast<int16_t>(raw) - CHANNEL_CENTER;
            next.channels[i] = value >= -CHANNEL_DEADZONE && value <= CHANNEL_DEADZONE ? 0 : value;
        }
        const auto mode = detail::DecodeBits(buffer_.data(), 60, 2);
        if (mode > 2) {
            return false;
        }
        next.mode = static_cast<Vt13Mode>(mode);
        next.pause = detail::DecodeBits(buffer_.data(), 62, 1) != 0;
        next.left_button = detail::DecodeBits(buffer_.data(), 63, 1) != 0;
        next.right_button = detail::DecodeBits(buffer_.data(), 64, 1) != 0;
        next.trigger = detail::DecodeBits(buffer_.data(), 76, 1) != 0;
        next.mouse.x = detail::DecodeInt16LE(buffer_.data() + 10);
        next.mouse.y = detail::DecodeInt16LE(buffer_.data() + 12);
        next.mouse.z = detail::DecodeInt16LE(buffer_.data() + 14);
        // Each mouse button occupies two bits, but only 0 and 1 are valid.
        const auto left = detail::DecodeBits(buffer_.data(), 128, 2);
        const auto right = detail::DecodeBits(buffer_.data(), 130, 2);
        const auto middle = detail::DecodeBits(buffer_.data(), 132, 2);
        if (left > 1 || right > 1 || middle > 1) {
            return false;
        }
        next.mouse.left = left != 0;
        next.mouse.right = right != 0;
        next.mouse.middle = middle != 0;
        next.keyboard.code = detail::DecodeUint16LE(buffer_.data() + 17);
        next.crc16 = detail::DecodeUint16LE(buffer_.data() + 19);
        state_ = next;
        CommitFrame(now_ms, false);
        return true;
    }
}  // namespace remote
