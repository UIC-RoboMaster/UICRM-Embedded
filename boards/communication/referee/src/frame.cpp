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
 # the implied warranty of MERCHANTABILITY or FITNESS FOR   #
 # A PARTICULAR PURPOSE.  See the GNU General Public License#
 # for more details.                                        #
 #                                                          #
 # You should have received a copy of the GNU General       #
 # Public License along with this program.  If not, see     #
 # <https://www.gnu.org/licenses/>.                         #
 ###########################################################*/

#include "frame.h"

#include <string.h>

#include "crc.h"

namespace referee_comm {

    size_t Encode(
        uint8_t* out, size_t capacity, uint8_t seq, uint16_t cmd_id, const void* payload, uint16_t length
    ) {
        if (out == nullptr || length > MAX_PAYLOAD)
            return 0;
        const size_t total = HEADER_SIZE + CMD_ID_SIZE + static_cast<size_t>(length) + TAIL_SIZE;
        if (total > capacity)
            return 0;

        out[0] = SOF;
        out[1] = static_cast<uint8_t>(length & 0xFF);
        out[2] = static_cast<uint8_t>((length >> 8) & 0xFF);
        out[3] = seq;
        Append_CRC8_Check_Sum(out, HEADER_SIZE);

        out[HEADER_SIZE] = static_cast<uint8_t>(cmd_id & 0xFF);
        out[HEADER_SIZE + 1] = static_cast<uint8_t>((cmd_id >> 8) & 0xFF);
        if (length > 0 && payload != nullptr)
            memcpy(out + HEADER_SIZE + CMD_ID_SIZE, payload, length);

        Append_CRC16_Check_Sum(out, static_cast<uint32_t>(total));
        return total;
    }

    void Decoder::Feed(const uint8_t* data, size_t size, Handler handler, void* ctx) {
        if (data == nullptr || size == 0)
            return;

        for (size_t i = 0; i < size; ++i) {
            const uint8_t byte = data[i];
            if (used_ == 0 && byte != SOF)
                continue;

            buffer_[used_++] = byte;

            while (used_ > 0) {
                if (buffer_[0] != SOF) {
                    Resync();
                    continue;
                }

                if (used_ < HEADER_SIZE)
                    break;
                const size_t payload = static_cast<size_t>(buffer_[1]) | (static_cast<size_t>(buffer_[2]) << 8);
                if (!Verify_CRC8_Check_Sum(buffer_, HEADER_SIZE) || payload > MAX_PAYLOAD) {
                    Resync();
                    continue;
                }
                const size_t total = HEADER_SIZE + CMD_ID_SIZE + payload + TAIL_SIZE;
                if (used_ < total)
                    break;
                if (!Verify_CRC16_Check_Sum(buffer_, static_cast<uint32_t>(total))) {
                    Resync();
                    continue;
                }

                Frame frame{};
                frame.cmd_id = static_cast<uint16_t>(buffer_[5]) | (static_cast<uint16_t>(buffer_[6]) << 8);
                frame.data = buffer_ + HEADER_SIZE + CMD_ID_SIZE;
                frame.length = static_cast<uint16_t>(payload);
                if (handler != nullptr)
                    handler(ctx, frame, buffer_, total);
                Discard(total);
            }
        }
    }

    void Decoder::Discard(size_t size) {
        used_ -= size;
        if (used_ > 0)
            memmove(buffer_, buffer_ + size, used_);
    }

    void Decoder::Resync() {
        size_t next = 1;
        while (next < used_) {
            if (buffer_[next] == SOF)
                break;
            ++next;
        }
        Discard(next);
    }

}  // namespace referee_comm
