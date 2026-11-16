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

#include "host.h"

#include <string.h>

#include "bsp_os.h"
#include "cmsis_os.h"
#include "task.h"

namespace host {

    namespace {
        static_assert(
            sizeof(host_gimbal_status_t) + referee::HEADER_SIZE + referee::CMD_ID_SIZE +
                    referee::TAIL_SIZE <=
                TX_CAPACITY,
            "TX_CAPACITY 装不下最大的上行帧"
        );

        /** @brief 拒绝 NaN/Inf 与明显越界的浮点，避免污染控制环。 */
        bool Finite(float value) {
            return value == value && value > -1e6f && value < 1e6f;
        }
    }  // namespace

    Host::Host(ByteLink& link) : link_(link) {
        link_.SetRxHandler(OnRx, this);
    }

    size_t Host::Pump(uint32_t now_ms) {
        const uint32_t bytes_before = stats_.tx_bytes;

        // 上一帧没写完时只能续写，插入新帧会让对端丢失帧边界。
        if (!Flush()) {
            ++stats_.tx_retry;
            return stats_.tx_bytes - bytes_before;
        }

        // 姿态流延迟敏感，先于转发与其他状态发出。
        EmitDue(0, now_ms);

        const uint32_t used = stats_.tx_bytes - bytes_before;
        DrainForward(used >= TICK_BUDGET ? 0 : TICK_BUDGET - used);

        for (size_t i = 1; i < SCHEDULE_COUNT; ++i) {
            if (stats_.tx_bytes - bytes_before >= TICK_BUDGET)
                break;
            EmitDue(i, now_ms);
        }

        return stats_.tx_bytes - bytes_before;
    }

    void Host::EmitDue(size_t index, uint32_t now_ms) {
        Schedule& item = schedule_[index];
        if (now_ms - item.last_ms < item.period_ms)
            return;
        item.last_ms = now_ms;
        (void)Emit(item.cmd);
    }

    bool Host::PostForward(const uint8_t* frame, size_t size) {
        if (frame == nullptr || size == 0 || size > referee::MAX_PACKET) {
            ++stats_.forward_dropped;
            return false;
        }
        const uint32_t head = forward_head_;
        if (head - forward_tail_ >= FORWARD_DEPTH) {
            ++stats_.forward_dropped;
            return false;
        }

        ForwardSlot& slot = forward_[head % FORWARD_DEPTH];
        slot.length = static_cast<uint16_t>(size);
        memcpy(slot.data, frame, size);

        // 先拷贝后发布，消费者只在 head 之前取槽位；临界区只保护下标更新。
        taskENTER_CRITICAL();
        forward_head_ = head + 1;
        taskEXIT_CRITICAL();
        return true;
    }

    bool Host::AimValid() const {
        return aim_ms_ != 0 && bsp::GetHighresTickMilliSec() - aim_ms_ < AIM_TIMEOUT_MS;
    }

    bool Host::NavigationValid() const {
        return navigation_ms_ != 0 && bsp::GetHighresTickMilliSec() - navigation_ms_ < NAVIGATION_TIMEOUT_MS;
    }

    bool Host::IsOnline() const {
        return last_rx_ms_ != 0 && bsp::GetHighresTickMilliSec() - last_rx_ms_ < LINK_TIMEOUT_MS;
    }

    void Host::Feed(const uint8_t* data, size_t size) {
        if (data == nullptr || size == 0)
            return;
        last_rx_ms_ = bsp::GetHighresTickMilliSec();
        decoder_.Feed(data, size, OnFrame, this);
    }

    void Host::OnRx(void* ctx, const uint8_t* data, size_t size) {
        static_cast<Host*>(ctx)->Feed(data, size);
    }

    void Host::OnFrame(void* ctx, const referee::Frame& frame, const uint8_t* raw, size_t size) {
        (void)raw;
        (void)size;
        Host* self = static_cast<Host*>(ctx);
        const uint32_t now = bsp::GetHighresTickMilliSec();

        switch (frame.cmd_id) {
            case AIM_CMD: {
                if (frame.length != sizeof(host_aim_cmd_t)) {
                    ++self->stats_.rx_rejected;
                    return;
                }
                host_aim_cmd_t value{};
                memcpy(&value, frame.data, sizeof(value));
                if (!Finite(value.pitch) || !Finite(value.yaw) || value.shoot > 1) {
                    ++self->stats_.rx_rejected;
                    return;
                }
                self->aim = value;
                self->aim_ms_ = now;
                ++self->stats_.rx_frames;
                return;
            }
            case NAVIGATION_CMD: {
                if (frame.length != sizeof(host_navigation_cmd_t)) {
                    ++self->stats_.rx_rejected;
                    return;
                }
                host_navigation_cmd_t value{};
                memcpy(&value, frame.data, sizeof(value));
                if (!Finite(value.vx) || !Finite(value.vy) || !Finite(value.wz) || !Finite(value.target_angle)) {
                    ++self->stats_.rx_rejected;
                    return;
                }
                self->navigation = value;
                self->navigation_ms_ = now;
                ++self->stats_.rx_frames;
                return;
            }
            default:
                // 未知命令静默跳过，字节流不受影响。
                return;
        }
    }

    bool Host::Emit(uint16_t cmd) {
        if (tx_length_ != 0)
            return false;

        // 交替使用两块小缓冲：USB 发送期间仍引用调用方指针，不能立刻覆写。
        tx_index_ ^= 1;
        const size_t length = Build(cmd, tx_buffer_[tx_index_], TX_CAPACITY);
        if (length == 0)
            return false;

        tx_ptr_ = tx_buffer_[tx_index_];
        tx_length_ = static_cast<uint16_t>(length);
        tx_offset_ = 0;
        tx_forward_ = false;
        return Flush();
    }

    bool Host::Flush() {
        while (tx_ptr_ != nullptr && tx_offset_ < tx_length_) {
            const int written = link_.Write(tx_ptr_ + tx_offset_, tx_length_ - tx_offset_);
            if (written <= 0)
                return false;
            tx_offset_ = static_cast<uint16_t>(tx_offset_ + written);
            stats_.tx_bytes += static_cast<uint32_t>(written);
        }
        if (tx_length_ != 0) {
            tx_length_ = 0;
            tx_offset_ = 0;
            tx_ptr_ = nullptr;
            ++stats_.tx_frames;
            if (tx_forward_) {
                // 转发帧直接从槽位发出，写完才允许生产者复用该槽。
                tx_forward_ = false;
                forward_tail_ = forward_tail_ + 1;
            }
        }
        return true;
    }

    void Host::DrainForward(size_t byte_limit) {
        size_t written = 0;
        while (forward_tail_ != forward_head_ && written < byte_limit) {
            if (tx_length_ != 0)
                return;

            const ForwardSlot& slot = forward_[forward_tail_ % FORWARD_DEPTH];
            tx_ptr_ = slot.data;
            tx_length_ = slot.length;
            tx_offset_ = 0;
            tx_forward_ = true;

            const uint32_t before = stats_.tx_bytes;
            const bool complete = Flush();
            written += stats_.tx_bytes - before;
            ++stats_.forwarded;
            if (!complete)
                return;
        }
    }

    size_t Host::Build(uint16_t cmd, uint8_t* out, size_t capacity) {
        switch (cmd) {
            case HOST_VERSION:
                return referee::Encode(out, capacity, seq_++, cmd, &version, sizeof(version));
            case ROBOT_STATUS:
                return referee::Encode(out, capacity, seq_++, cmd, &robot_status, sizeof(robot_status));
            case CHASSIS_STATUS:
                return referee::Encode(out, capacity, seq_++, cmd, &chassis_status, sizeof(chassis_status));
            case GIMBAL_STATUS:
                return referee::Encode(out, capacity, seq_++, cmd, &gimbal_status, sizeof(gimbal_status));
            case SHOOT_STATUS:
                return referee::Encode(out, capacity, seq_++, cmd, &shoot_status, sizeof(shoot_status));
            default:
                return 0;
        }
    }

}  // namespace host
