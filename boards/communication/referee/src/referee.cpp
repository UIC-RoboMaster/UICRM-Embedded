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

#include "referee.h"

#include <string.h>

#include "bsp_os.h"
#include "crc.h"

namespace referee_comm {

    int Protocol::OnBuild(uint16_t cmd_id, uint8_t* out, size_t capacity) {
        (void)cmd_id;
        (void)out;
        (void)capacity;
        return -1;
    }

    Referee::Referee(bsp::UART* uart, Protocol& protocol, bsp::UART* forward_to, uint32_t online_threshold_ms)
        : uart_(uart), protocol_(protocol), forward_(forward_to), online_threshold_(online_threshold_ms) {
        uart_->SetupRxData(&read_ptr_, &read_len_);
        uart_->RegisterCallback(CallbackWrapper, this);

        bsp::thread_init_t thread_init = {
            .func = ThreadFunc,
            .args = this,
            .attr = thread_attr_,
        };
        thread_ = new bsp::EventThread(thread_init);
    }

    Referee::~Referee() {
        delete thread_;
    }

    size_t Referee::Encode(
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

    void Referee::Feed(const uint8_t* data, size_t size) {
        if (data == nullptr || size == 0)
            return;

        // 与既有实现一致：整段输入即视为对端存活证据，与帧本身是否有效无关。
        Heartbeat();

        for (size_t i = 0; i < size; ++i) {
            const uint8_t byte = data[i];
            if (used_ == 0 && byte != SOF)
                continue;

            // 等待最长候选帧的分支已保证上界，此处仅作防御。
            if (used_ >= MAX_PACKET)
                Discard(1);
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
                protocol_.OnCommand(frame);
                Forward(buffer_, total);
                Discard(total);
            }
        }
    }

    size_t Referee::Transmit(uint16_t cmd_id) {
        uint8_t* const data = tx_ + HEADER_SIZE + CMD_ID_SIZE;
        const int length = protocol_.OnBuild(cmd_id, data, MAX_PAYLOAD);
        if (length < 0 || static_cast<size_t>(length) > MAX_PAYLOAD)
            return 0;

        const size_t total = HEADER_SIZE + CMD_ID_SIZE + static_cast<size_t>(length) + TAIL_SIZE;
        tx_[0] = SOF;
        tx_[1] = static_cast<uint8_t>(length & 0xFF);
        tx_[2] = static_cast<uint8_t>((length >> 8) & 0xFF);
        tx_[3] = seq_++;
        Append_CRC8_Check_Sum(tx_, HEADER_SIZE);

        tx_[HEADER_SIZE] = static_cast<uint8_t>(cmd_id & 0xFF);
        tx_[HEADER_SIZE + 1] = static_cast<uint8_t>((cmd_id >> 8) & 0xFF);

        Append_CRC16_Check_Sum(tx_, static_cast<uint32_t>(total));
        return Write(tx_, total);
    }

    void Referee::Reset() {
        used_ = 0;
    }

    bool Referee::IsOnline() const {
        if (last_uptime_ == 0)
            return false;
        return bsp::GetHighresTickMilliSec() - last_uptime_ < online_threshold_;
    }

    uint32_t Referee::GetLastUptime() const {
        return last_uptime_;
    }

    uint32_t Referee::GetLastUptimeMicrosec() const {
        return last_uptime_microsec_;
    }

    void Referee::SetThreshold(uint32_t threshold) {
        online_threshold_ = threshold;
    }

    void Referee::Heartbeat() {
        last_uptime_ = bsp::GetHighresTickMilliSec();
        last_uptime_microsec_ = bsp::GetHighresTickMicroSec();
    }

    void Referee::SetForwardTarget(bsp::UART* target) {
        forward_ = target;
    }

    void Referee::Forward(const uint8_t* data, size_t size) {
        if (forward_ == nullptr || data == nullptr || size == 0)
            return;
        // 尽力而为：转发不得让接收线程因下游拥塞而阻塞，写不下即丢弃该帧。
        // TODO: 本处在接收线程内直接写串口，与 minipc_task 的 Host::Transmit 竞争同一
        // TODO: 发送缓冲。后续改为投递队列，由单一写者线程统一发送裁判帧与 host 帧。
        (void)forward_->Write<false>(data, static_cast<uint32_t>(size));
    }

    void Referee::Discard(size_t size) {
        if (size > used_)
            size = used_;
        used_ -= size;
        if (size > 0 && used_ > 0)
            memmove(buffer_, buffer_ + size, used_);
    }

    void Referee::Resync() {
        size_t next = 1;
        while (next < used_) {
            const uint8_t byte = buffer_[next];
            if (byte == SOF)
                break;
            ++next;
        }
        Discard(next);
    }

    size_t Referee::Write(const uint8_t* data, size_t size) {
        if (uart_ == nullptr)
            return 0;
        const int32_t written = uart_->Write(data, static_cast<uint32_t>(size));
        return written > 0 ? static_cast<size_t>(written) : 0;
    }

    void Referee::CallbackWrapper(void* args) {
        // 回调在中断上下文，只唤醒线程，分帧与解包在 ThreadFunc 中完成。
        reinterpret_cast<Referee*>(args)->thread_->Set();
    }

    void Referee::ThreadFunc(void* args) {
        Referee* referee = reinterpret_cast<Referee*>(args);
        referee->Feed(referee->read_ptr_, referee->read_len_);
    }

}  // namespace referee_comm
