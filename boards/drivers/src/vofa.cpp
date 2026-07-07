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

#include "vofa.h"

#include <cstring>

namespace driver {

    Vofa::Vofa(bsp::UART* uart, uint32_t frame_tail) {
        Attach(uart, frame_tail);
    }

    void Vofa::Attach(bsp::UART* uart, uint32_t frame_tail) {
        uart_ = uart;
        frame_tail_ = frame_tail;
    }

    void Vofa::BindChannel(uint8_t index, const float* data) {
        if (index >= MaxChannels)
            return;
        channels_[index] = data;
        if (index >= channel_count_)
            channel_count_ = index + 1;
    }

    void Vofa::BindChannels(const float* const* channels, uint8_t count) {
        ClearChannels();
        if (channels == nullptr)
            return;

        if (count > MaxChannels)
            count = MaxChannels;

        for (uint8_t i = 0; i < count; i++)
            channels_[i] = channels[i];
        channel_count_ = count;
    }

    void Vofa::ClearChannels() {
        for (uint8_t i = 0; i < MaxChannels; i++)
            channels_[i] = nullptr;
        channel_count_ = 0;
    }

    uint8_t Vofa::ChannelCount() const {
        return channel_count_;
    }

    void Vofa::Send() {
        if (uart_ == nullptr || channel_count_ == 0)
            return;

        PackFrame();
        const uint32_t len = channel_count_ * sizeof(float) + sizeof(uint32_t);
        uart_->Write(tx_buffer_, len);
    }

    void Vofa::SetupRx(const char** names, uint8_t count, uint32_t buffer_size) {
        rx_names_ = names;
        rx_name_count_ = count;
        if (uart_ == nullptr)
            return;

        uart_->SetupRx(buffer_size);
        uart_->SetupRxData(&rx_ptr_, &rx_len_);
        uart_->RegisterCallback(&Vofa::UartCallback, this);
    }

    void Vofa::UartCallback(void* ctx) {
        auto* self = static_cast<Vofa*>(ctx);
        if (self->rx_len_ > 0)
            self->HandleRx(self->rx_ptr_, static_cast<uint16_t>(self->rx_len_));
    }

    void Vofa::HandleRx(const uint8_t* data, uint16_t length) {
        uint16_t copy_len = length;
        if (copy_len > RxBufferSize)
            copy_len = RxBufferSize;
        memcpy(rx_buffer_, data, copy_len);
        if (copy_len < RxBufferSize)
            rx_buffer_[copy_len] = 0;
        ParseRx(copy_len);
    }

    int32_t Vofa::VariableIndex() const {
        return variable_index_;
    }

    float Vofa::VariableValue() const {
        return variable_value_;
    }

    void Vofa::ParseRx(uint16_t length) {
        const int value_offset = MatchVariableName(length);
        ParseVariableValue(length, value_offset);
    }

    uint8_t Vofa::MatchVariableName(uint16_t length) {
        char name[RxNameMaxLength];
        int index = 0;

        for (index = 0; rx_buffer_[index] != '=' && index < length && rx_buffer_[index] != 0; index++)
            name[index] = static_cast<char>(rx_buffer_[index]);
        name[index] = 0;

        for (uint8_t i = 0; i < rx_name_count_; i++) {
            if (rx_names_ != nullptr && strcmp(name, rx_names_[i]) == 0) {
                variable_index_ = static_cast<int32_t>(i);
                return static_cast<uint8_t>(index + 1);
            }
        }

        variable_index_ = -1;
        return static_cast<uint8_t>(index + 1);
    }

    void Vofa::ParseVariableValue(uint16_t length, int offset) {
        int dot_offset = 0;
        int sign = 1;
        int i = 0;

        variable_value_ = 0.f;
        if (variable_index_ == -1)
            return;

        if (rx_buffer_[offset] == '-') {
            sign = -1;
            offset++;
        }

        for (i = offset; rx_buffer_[i] != '#' && i < length && rx_buffer_[i] != 0; i++) {
            if (rx_buffer_[i] == '.') {
                dot_offset = i;
            } else {
                variable_value_ = variable_value_ * 10.f + static_cast<float>(rx_buffer_[i] - '0');
            }
        }

        if (dot_offset != 0) {
            float divisor = 1.f;
            for (int j = dot_offset + 1; j < i; j++)
                divisor *= 10.f;
            variable_value_ /= divisor;
        }

        variable_value_ *= static_cast<float>(sign);
    }

    void Vofa::PackFrame() {
        memset(tx_buffer_, 0, sizeof(tx_buffer_));

        for (uint8_t i = 0; i < channel_count_; i++) {
            if (channels_[i] != nullptr)
                memcpy(tx_buffer_ + i * sizeof(float), channels_[i], sizeof(float));
        }

        memcpy(tx_buffer_ + channel_count_ * sizeof(float), &frame_tail_, sizeof(uint32_t));
    }

}  // namespace driver
