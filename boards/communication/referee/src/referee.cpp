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

#include "bsp_os.h"

namespace referee {

    int Protocol::OnBuild(uint16_t cmd_id, uint8_t* out, size_t capacity) {
        (void)cmd_id;
        (void)out;
        (void)capacity;
        return -1;
    }

    Referee::Referee(bsp::UART* uart, Protocol& protocol, ForwardSink* forward_to, uint32_t online_threshold_ms)
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

    void Referee::Feed(const uint8_t* data, size_t size) {
        if (data == nullptr || size == 0)
            return;

        // 与既有实现一致：整段输入即视为对端存活证据，与帧本身是否有效无关。
        Heartbeat();
        decoder_.Feed(data, size, OnFrame, this);
    }

    void Referee::OnFrame(void* ctx, const Frame& frame, const uint8_t* raw, size_t size) {
        Referee* self = static_cast<Referee*>(ctx);
        self->protocol_.OnCommand(frame);
        if (self->forward_ != nullptr)
            (void)self->forward_->PostForward(raw, size);
    }

    size_t Referee::Transmit(uint16_t cmd_id) {
        uint8_t* const data = tx_ + HEADER_SIZE + CMD_ID_SIZE;
        const int length = protocol_.OnBuild(cmd_id, data, MAX_PAYLOAD);
        if (length < 0)
            return 0;

        return Transmit(cmd_id, data, static_cast<size_t>(length));
    }

    size_t Referee::Transmit(uint16_t cmd_id, const void* payload, size_t length) {
        if (length > MAX_PAYLOAD)
            return 0;

        const size_t total = Encode(tx_, sizeof(tx_), seq_, cmd_id, payload, static_cast<uint16_t>(length));
        if (total == 0)
            return 0;

        ++seq_;
        return Write(tx_, total);
    }

    bool Referee::IsOnline() const {
        if (last_uptime_ == 0)
            return false;
        return bsp::GetHighresTickMilliSec() - last_uptime_ < online_threshold_;
    }

    void Referee::Heartbeat() {
        last_uptime_ = bsp::GetHighresTickMilliSec();
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

}  // namespace referee
