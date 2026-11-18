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
#include "frame.h"

namespace referee {

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
         * @brief 根据命令码构造待发送的数据段。
         * @param[in] cmd_id 待构造的数据帧命令码。
         * @param[out] out 输出缓冲区，用于写入构造后的数据段。
         * @param[in] capacity 输出缓冲区的可用容量，单位为字节。
         * @return 成功时返回实际写入的数据段长度；命令不受支持或缓冲区容量不足时返回 -1。
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
         * @param[in] forward_to 可选的下游接收者，非空时把校验通过的原始帧原样投递，
         *            由接收者决定排队与发送时机。转发为尽力而为，投递失败即丢弃该帧，
         *            不阻塞接收线程，也不影响本地解包。
         * @param[in] online_threshold_ms 未收到任何字节的离线判定阈值，单位毫秒。
         * @note 对象须覆盖接收中断使用期。VT13 遥控帧由 controller 侧独立处理。
         * @warning forward_to 在接收线程中被调用，其实现不得阻塞。
         */
        explicit Referee(
            bsp::UART* uart, Protocol& protocol, ForwardSink* forward_to = nullptr, uint32_t online_threshold_ms = 250
        );

        ~Referee();

        Referee(const Referee&) = delete;
        Referee& operator=(const Referee&) = delete;

        /**
         * @brief 输入任意长度字节流，完成切帧、校验与分发。
         * @param[in] data 接收字节地址，空指针直接返回。
         * @param[in] size 字节数。
         * @note 支持半包、粘包、连续帧及校验失败后的后缀恢复。
         */
        void Feed(const uint8_t* data, size_t size);

        /**
         * @brief 通过 Protocol::OnBuild 构造数据段，再封帧发送。
         * @param[in] cmd_id 命令码。
         * @return 串口接受的字节数；构造失败或未写入时为 0。
         */
        size_t Transmit(uint16_t cmd_id);

        /**
         * @brief 直接把调用方构造的数据段封帧发送，不调用 Protocol::OnBuild。
         * @param[in] cmd_id 外层命令码，例如 UI 绘图使用 0x0301。
         * @param[in] payload 完整 data 段，不含外层帧头、命令码和 CRC；仅在调用期间读取。
         * @param[in] length data 段的实际字节数，不能超过 MAX_PAYLOAD；为 0 时 payload 可为空。
         * @return 串口接受的字节数（含帧开销，可能短写）；参数非法或未写入时为 0。
         * @note 0x0301 须包含 6 字节交互头和实际内容，协议相关的长度与频率限制由调用方保证。
         */
        size_t Transmit(uint16_t cmd_id, const void* payload, size_t length);

        /**
         * @brief 设置收到有效裁判帧后，把数据交给哪个对象。
         * @details 每收到一帧通过 CRC 校验的完整数据，就调用
         * sink->PostForward()，将帧头、数据和校验尾原样交给它。
         * 即使本地协议不认识该帧，也仍会转发。
         * @param[in] sink 接收帧的对象；传入 nullptr 表示停止转发，
         * 本地协议仍会继续解析收到的数据。
         * @warning sink 必须保持有效。请在开始接收前设置；开始接收后，
         * 只能确认接收处理已停止时再更换。
         */
        void SetForwardSink(ForwardSink* sink) {
            forward_ = sink;
        }

        /** @brief 最近一次收到字节是否在阈值内。 */
        bool IsOnline() const;

      private:
        /** @brief 记录一次对端存活证据。 */
        void Heartbeat();
        /** @brief 写出完整帧。 */
        size_t Write(const uint8_t* data, size_t size);
        /** @brief 分帧回调：本地解包 + 转发投递。 */
        static void OnFrame(void* ctx, const Frame& frame, const uint8_t* raw, size_t size);

        static void CallbackWrapper(void* args);
        static void ThreadFunc(void* args);

        bsp::UART* uart_;
        Protocol& protocol_;
        ForwardSink* forward_ = nullptr;  ///< 原始帧转发目标，为空时不转发。

        Decoder decoder_;              ///< 分帧与校验，半包状态由它持有。
        uint8_t seq_ = 0;              ///< 发送包序号。
        uint8_t tx_[MAX_PACKET] = {};  ///< 发送缓存。

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

        volatile uint32_t last_uptime_ = 0;  ///< 最近一次收到字节，单位毫秒。
        uint32_t online_threshold_ = 250;    ///< 离线判定阈值，单位毫秒。
    };

}  // namespace referee
