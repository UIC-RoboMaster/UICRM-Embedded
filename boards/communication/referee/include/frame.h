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

namespace referee {

    /*
     * 裁判系统线格式（2018~2027 各赛季一致，改动需先核实官方规则）：
     *
     *   frame_header            cmd_id   data      frame_tail
     *   SOF(1) len(2) seq(1)    (2)      (n)       CRC16(2)
     *   其中 len 为 data 字节数，frame_header 尾部含 CRC8(1)。
     *
     * 裁判转发（MCU → minipc）与 host 双向帧共用本格式：编码统一走 Encode，
     * 解码各自持有 Decoder，互不影响。
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

    /** @brief 已通过 CRC16 校验的完整数据帧视图，生命周期止于回调返回。 */
    struct Frame {
        uint16_t cmd_id = 0;            ///< 命令码。
        const uint8_t* data = nullptr;  ///< data 段首地址，指向解码器内部缓存。
        uint16_t length = 0;            ///< data 段字节数。
    };

    /**
     * @brief 校验通过帧的下游接收者。
     * @details 接收者只关心「一帧完整字节」，不关心链路类型；
     * 由 host 侧实现，具体排队与发送时机由实现方决定。
     */
    class ForwardSink {
      public:
        virtual ~ForwardSink() = default;

        /**
         * @brief 投递一帧已通过 CRC16 的完整帧。
         * @param[in] frame 整帧字节（含帧头与 CRC16）。
         * @param[in] size 整帧字节数。
         * @return 接收方已留存该帧时为 true；队列满返回 false，调用方不得阻塞等待。
         * @note 在接收线程中调用，实现方不得在此阻塞。
         */
        virtual bool PostForward(const uint8_t* frame, size_t size) = 0;
    };

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
    size_t Encode(uint8_t* out, size_t capacity, uint8_t seq, uint16_t cmd_id, const void* payload, uint16_t length);

    /**
     * @brief 字节流分帧与 CRC 校验。
     * @details 支持半包、粘包、连续帧及校验失败后的后缀恢复。
     * 并发：不可重入，须由单一上下文调用。
     */
    class Decoder {
      public:
        /**
         * @brief 一帧校验通过后的回调。
         * @param[in] raw 整帧字节（含帧头与 CRC16），回调返回后失效，需要留存须自行复制。
         * @param[in] size 整帧字节数。
         */
        using Handler = void (*)(void* ctx, const Frame& frame, const uint8_t* raw, size_t size);

        /**
         * @brief 输入任意长度字节流，完成切帧、校验与分发。
         * @param[in] data 接收字节地址，空指针直接返回。
         * @param[in] size 字节数。
         */
        void Feed(const uint8_t* data, size_t size, Handler handler, void* ctx);

      private:
        /** @brief 从缓存头部丢弃 size 字节。 */
        void Discard(size_t size);
        /** @brief 丢弃到下一个可能的帧头，用于校验失败后的边界恢复。 */
        void Resync();

        uint8_t buffer_[MAX_PACKET] = {};  ///< 分帧缓存。
        size_t used_ = 0;                  ///< 缓存中待处理字节数。
    };

}  // namespace referee
