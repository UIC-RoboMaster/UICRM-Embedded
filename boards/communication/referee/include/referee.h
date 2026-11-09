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
#pragma once

#include <stddef.h>
#include <stdint.h>

#include "bsp_thread.h"
#include "bsp_uart.h"

namespace referee_comm {

    /*
     * 裁判系统线格式（2018~2027 各赛季一致，改动需先核实官方规则）：
     *
     *   frame_header            cmd_id   data      frame_tail
     *   SOF(1) len(2) seq(1)    (2)      (n)       CRC16(2)
     *   其中 len 为 data 字节数，frame_header 尾部含 CRC8(1)。
     */

    /** @brief 数据帧起始字节。 */
    constexpr uint8_t SOF = 0xA5;

    /** @brief 帧头长度：SOF(1) + len(2) + seq(1) + CRC8(1)。 */
    constexpr size_t HEADER_SIZE = 5;
    /** @brief 命令码长度。 */
    constexpr size_t CMD_ID_SIZE = 2;
    /** @brief 帧尾长度：CRC16(2)。 */
    constexpr size_t TAIL_SIZE = 2;

    /**
     * @brief 单帧最大容量，单位字节。
     * @details 需容纳 2027 赛季图传链路的 0x0310（300 字节数据段），
     * 因此不小于 300 + HEADER_SIZE + CMD_ID_SIZE + TAIL_SIZE。
     */
    constexpr size_t MAX_PACKET = 320;
    /** @brief data 段最大字节数。 */
    constexpr size_t MAX_PAYLOAD = MAX_PACKET - HEADER_SIZE - CMD_ID_SIZE - TAIL_SIZE;

    /** @brief 已通过 CRC16 校验的完整数据帧视图，生命周期止于 Feed 返回。 */
    struct Frame {
        uint16_t cmd_id = 0;            ///< 命令码。
        const uint8_t* data = nullptr;  ///< data 段首地址，指向内部接收缓存。
        uint16_t length = 0;            ///< data 段字节数。
    };

    /**
     * @brief 赛季相关的载荷解码与编码接口。
     * @details 线格式由 Referee 保证，本接口只承担 cmd_id 到字段的映射，
     * 因此每个赛季独立实现一份，互不影响。
     */
    class Protocol {
      public:
        virtual ~Protocol() = default;

        /**
         * @brief 处理一帧已通过校验的数据。
         * @param[in] frame 完整帧视图，不得在返回后保留指针。
         * @return 命令码被本实现识别并处理时为 true。
         * @note 返回 false 表示未知或已废弃命令，不得据此丢弃字节流。
         */
        virtual bool OnCommand(const Frame& frame) = 0;

        /**
         * @brief 按命令码生成待发送的 data 段。
         * @param[in] cmd_id 命令码。
         * @param[out] out data 段写入地址，容量至少为 capacity。
         * @param[in] capacity 可用字节数。
         * @return 写入的字节数；不发送该命令时返回 -1。
         */
        virtual int OnBuild(uint16_t cmd_id, uint8_t* out, size_t capacity);
    };

    /**
     * @brief 裁判系统数据接收与解包。
     * @details 绑定一路串口，完成字节流分帧、CRC 校验、重同步与在线判定，
     * 再把 data 段交给赛季 Protocol 解包。线格式与赛季无关，字段随赛季替换。
     * 接收回调只唤醒线程，分帧与解包在独立线程中完成，不占用中断上下文。
     * 并发：Feed、Transmit 不可重入，须由单一上下文串行调用。
     */
    class Referee {
      public:
        /**
         * @brief 绑定已配置的串口并启动接收线程。
         * @param[in] uart 串口实例，不得为空，须先完成 SetupRx/SetupTx。
         * @param[in] protocol 赛季解包实现，生命周期须覆盖本对象。
         * @param[in] forward_to 可选的下游串口，非空时把校验通过的原始帧原样写出，
         *            供上位机按官方协议自行解析。上游帧与上位机帧的 cmd_id 不重叠，
         *            可共用同一条串口。转发为尽力而为，写不下即丢弃，不阻塞接收线程。
         * @param[in] online_threshold_ms 未收到任何字节的离线判定阈值，单位毫秒。
         * @note 对象须覆盖接收中断使用期。VT13 遥控帧由 controller 侧独立处理。
         * @warning forward_to 在接收线程中被写入，调用方不得在其他线程同时写该串口。
         */
        explicit Referee(
            bsp::UART* uart, Protocol& protocol, bsp::UART* forward_to = nullptr, uint32_t online_threshold_ms = 250
        );

        ~Referee();

        Referee(const Referee&) = delete;
        Referee& operator=(const Referee&) = delete;

        /**
         * @brief 将 data 段封装为完整帧。
         * @param[out] out 目标缓冲，至少 capacity 字节。
         * @param[in] capacity 目标缓冲容量。
         * @param[in] seq 包序号，由调用方维护。
         * @param[in] cmd_id 命令码。
         * @param[in] payload data 段地址，length 为 0 时可为空。
         * @param[in] length data 段字节数。
         * @return 完整帧字节数；参数非法时返回 0。
         */
        static size_t Encode(
            uint8_t* out, size_t capacity, uint8_t seq, uint16_t cmd_id, const void* payload, uint16_t length
        );

        /**
         * @brief 输入任意长度字节流，完成切帧、校验与分发。
         * @param[in] data 接收字节地址，空指针直接返回。
         * @param[in] size 字节数。
         * @note 支持半包、粘包、连续帧及校验失败后的后缀恢复。
         */
        void Feed(const uint8_t* data, size_t size);

        /**
         * @brief 按命令码向对端发送一帧。
         * @param[in] cmd_id 命令码。
         * @return 实际写出的字节数；该命令无内容或写失败时为 0。
         */
        size_t Transmit(uint16_t cmd_id);

        /**
         * @brief 设置或取消原始帧转发目标。
         * @param[in] target 下游串口，传 nullptr 表示停止转发。
         * @warning 与构造函数相同：该串口不得被其他线程同时写入。
         */
        void SetForwardTarget(bsp::UART* target);

        /** @brief 丢弃暂存半包，保留离线判定时间与包序号。 */
        void Reset();

        /** @brief 最近一次收到字节是否在阈值内。 */
        bool IsOnline() const;
        /** @brief 最近一次收到字节的时间，单位毫秒。 */
        uint32_t GetLastUptime() const;
        /** @brief 最近一次收到字节的时间，单位微秒。 */
        uint32_t GetLastUptimeMicrosec() const;
        /** @brief 设置离线判定阈值，单位毫秒。 */
        void SetThreshold(uint32_t threshold);

      private:
        /** @brief 从缓存头部丢弃 size 字节。 */
        void Discard(size_t size);
        /** @brief 丢弃到下一个可能的帧头，用于校验失败后的边界恢复。 */
        void Resync();
        /** @brief 记录一次对端存活证据。 */
        void Heartbeat();
        /** @brief 写出完整帧。 */
        size_t Write(const uint8_t* data, size_t size);
        /**
         * @brief 把校验通过的原始帧原样转发到下游串口。
         * @warning 转发目标与 Host 共用同一条串口。当前由接收线程直接写出，
         * 与 minipc_task 线程里的 Host::Transmit 存在竞争（bsp::UART::Write 无锁）。
         */
        // TODO: 转发改走队列，由 minipc_task 单线程统一写出，
        // TODO: 让裁判帧与 host 帧共用同一条发送路径，消除并发写。
        void Forward(const uint8_t* data, size_t size);

        static void CallbackWrapper(void* args);
        static void ThreadFunc(void* args);

        bsp::UART* uart_;
        Protocol& protocol_;
        bsp::UART* forward_ = nullptr;  ///< 原始帧转发目标，为空时不转发。

        uint8_t buffer_[MAX_PACKET] = {};  ///< 接收与解帧共用缓存。
        size_t used_ = 0;                  ///< 缓存中待处理字节数。
        uint8_t seq_ = 0;                  ///< 发送包序号。
        uint8_t tx_[MAX_PACKET] = {};      ///< 发送缓存。

        uint8_t* read_ptr_ = nullptr;
        uint32_t read_len_ = 0;
        bsp::EventThread* thread_ = nullptr;

        const osThreadAttr_t thread_attr_ =
            {.name = "Referee",
             .attr_bits = osThreadDetached,
             .cb_mem = nullptr,
             .cb_size = 0,
             .stack_mem = nullptr,
             .stack_size = 256 * 4,
             .priority = (osPriority_t)osPriorityHigh,
             .tz_module = 0,
             .reserved = 0};

        volatile uint32_t last_uptime_ = 0;           ///< 最近一次收到字节，单位毫秒。
        volatile uint32_t last_uptime_microsec_ = 0;  ///< 最近一次收到字节，单位微秒。
        uint32_t online_threshold_ = 250;             ///< 离线判定阈值，单位毫秒。
    };

}  // namespace referee_comm
