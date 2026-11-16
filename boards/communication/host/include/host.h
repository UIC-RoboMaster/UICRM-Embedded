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
#pragma once

#include <stddef.h>
#include <stdint.h>

#include "frame.h"
#include "link.h"
#include "host_protocol.h"

namespace host {

    /** @brief 转发队列深度。1 kHz 轮询下一次只需吸收 1 ms 内的裁判突发，4 槽足够。 */
    constexpr size_t FORWARD_DEPTH = 4;
    /** @brief 单次 Pump 允许写出的字节数上限，避免转发帧挤占控制帧。 */
    constexpr size_t TICK_BUDGET = 512;
    /** @brief 上行帧缓冲容量，需容纳最大上行帧（0x0503 为 33 字节）。 */
    constexpr size_t TX_CAPACITY = 64;
    /** @brief 自瞄指令超时，超时后不得再据此开火。 */
    constexpr uint32_t AIM_TIMEOUT_MS = 100;
    /** @brief 导航指令超时，超时后底盘指令视为无效。 */
    constexpr uint32_t NAVIGATION_TIMEOUT_MS = 200;
    /** @brief minipc 在线判定阈值。 */
    constexpr uint32_t LINK_TIMEOUT_MS = 500;
    /** @brief 上行调度表项数。 */
    constexpr size_t SCHEDULE_COUNT = 5;

    /** @brief 上行调度表项，数组顺序即发送优先级。 */
    struct Schedule {
        uint16_t cmd;
        uint16_t period_ms;
        uint32_t last_ms = 0;
    };

    /**
     * @brief minipc 链路：上行调度、下行解码、裁判帧转发。
     * @details 本类是整条链路唯一的写者，裁判帧由 Referee 通过 PostForward 投递，
     * 与 host 自身帧共用同一个发送序列，不会交错或截断。
     * 不自建线程：由应用的一个 1 kHz 任务调用 Pump，周期与优先级由应用掌握。
     * 并发：Pump 只允许单一线程调用；PostForward 由裁判接收线程调用；
     * 下行状态在链路中断中写入，应用侧读取时以 Valid() 判断时效。
     */
    class Host final : public referee::ForwardSink {
      public:
        /** @brief 运行计数，用于台架观测链路质量。 */
        struct Stats {
            uint32_t tx_frames = 0;         ///< 已完整写出的帧数。
            uint32_t tx_bytes = 0;          ///< 已写出的字节数。
            uint32_t tx_retry = 0;          ///< 因链路忙而跨 tick 续写的次数。
            uint32_t forwarded = 0;         ///< 成功投递并转发的裁判帧数。
            uint32_t forward_dropped = 0;   ///< 转发队列满或超长丢弃的帧数。
            uint32_t rx_frames = 0;         ///< 校验通过的下行帧数。
            uint32_t rx_rejected = 0;       ///< 长度或数值非法的下行帧数。
        };

        /** @param[in] link 物理链路，生命周期须覆盖本对象。 */
        explicit Host(ByteLink& link);

        ~Host() override = default;

        Host(const Host&) = delete;
        Host& operator=(const Host&) = delete;

        /**
         * @brief 发送一轮：续写未完成帧、发出到期帧、排空转发队列。
         * @param[in] now_ms 当前毫秒时基，须单调递增（允许回绕）。
         * @return 本轮写出的字节数。
         * @note 由单一任务以约 1 kHz 调用，不得在多处同时调用。
         */
        size_t Pump(uint32_t now_ms);

        /**
         * @brief 投递一帧裁判原始帧，由 Referee 在接收线程调用。
         * @return 入队成功为 true；队列满或长度超限返回 false，帧被丢弃。
         */
        bool PostForward(const uint8_t* frame, size_t size) override;

        /* ===== 上行载荷：应用侧填，Pump 按调度表发送 ===== */
        host_version_t version{};            ///< 0x0500，1Hz
        host_robot_status_t robot_status{};  ///< 0x0501，1Hz
        host_chassis_status_t chassis_status{};  ///< 0x0502，100Hz
        host_gimbal_status_t gimbal_status{};    ///< 0x0503，500Hz
        host_shoot_status_t shoot_status{};      ///< 0x0504，50Hz

        /* ===== 下行结果：最近一帧有效载荷 ===== */
        host_aim_cmd_t aim{};                ///< 0x0400，超时后不得使用
        host_navigation_cmd_t navigation{};  ///< 0x0401，超时后不得使用

        /** @brief 自瞄指令是否在超时窗口内。 */
        bool AimValid() const;
        /** @brief 导航指令是否在超时窗口内。 */
        bool NavigationValid() const;
        /** @brief minipc 是否在线。 */
        bool IsOnline() const;

        const Stats& stats() const {
            return stats_;
        }

      private:
        void Feed(const uint8_t* data, size_t size);
        static void OnRx(void* ctx, const uint8_t* data, size_t size);

        /** @brief 下行帧分发，由 Decoder 回调。 */
        static void OnFrame(void* ctx, const referee::Frame& frame, const uint8_t* raw, size_t size);

        /** @brief 按命令码构造一帧并写出，上一帧未写完时返回 false。 */
        bool Emit(uint16_t cmd);
        /** @brief 续写当前帧，写完后清空并释放占用的转发槽位。 */
        bool Flush();
        /** @brief 在预算内尽量排空转发队列。 */
        void DrainForward(size_t byte_limit);
        /** @brief 构造 data 段，返回整帧长度。 */
        size_t Build(uint16_t cmd, uint8_t* out, size_t capacity);
        /** @brief 到期则发送表项对应的帧。 */
        void EmitDue(size_t index, uint32_t now_ms);

        ByteLink& link_;
        referee::Decoder decoder_;

        /** @brief 上行调度表，500Hz 姿态在最前，其余按周期放宽。 */
        Schedule schedule_[SCHEDULE_COUNT] = {
            {GIMBAL_STATUS, 2},    // 500Hz
            {CHASSIS_STATUS, 10},  // 100Hz
            {SHOOT_STATUS, 20},    // 50Hz
            {ROBOT_STATUS, 1000},  // 1Hz
            {HOST_VERSION, 1000},  // 1Hz
        };

        /** @brief 上行帧双缓冲：USB 发送期间仍引用调用方指针，交替使用避免覆写。 */
        uint8_t tx_buffer_[2][TX_CAPACITY] = {};
        uint8_t tx_index_ = 0;
        const uint8_t* tx_ptr_ = nullptr;  ///< 当前在发的帧，指向 tx_buffer_ 或转发槽位。
        uint16_t tx_length_ = 0;
        uint16_t tx_offset_ = 0;
        bool tx_forward_ = false;  ///< 当前帧来自转发队列，写完要释放槽位。
        uint8_t seq_ = 0;

        struct ForwardSlot {
            uint16_t length;
            uint8_t data[referee::MAX_PACKET];
        };
        ForwardSlot forward_[FORWARD_DEPTH] = {};
        volatile uint32_t forward_head_ = 0;  ///< 生产者写入位置。
        volatile uint32_t forward_tail_ = 0;  ///< 消费者读取位置。

        uint32_t aim_ms_ = 0;
        uint32_t navigation_ms_ = 0;
        volatile uint32_t last_rx_ms_ = 0;

        Stats stats_{};
    };

}  // namespace host
