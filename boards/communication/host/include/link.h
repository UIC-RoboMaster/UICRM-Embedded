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

namespace bsp {
    class UART;
#ifndef NO_USB
    class VirtualUSB;
#endif
}  // namespace bsp

namespace host {

    /**
     * @brief 面向字节流的物理链路。
     * @details Host 只依赖本接口，UART / USB 各自适配；CAN 等报文型链路
     * 需在上层做分片重组后同样表现为字节流。
     * 并发：Write 只允许 Host 的发送任务调用，SetRxHandler 只允许初始化时调用。
     */
    class ByteLink {
      public:
        /**
         * @brief 接收回调。
         * @param[in] data 本次收到的字节，回调返回后失效。
         * @note 在中断上下文调用，处理时间需与波特率匹配。
         */
        using RxHandler = void (*)(void* ctx, const uint8_t* data, size_t size);

        virtual ~ByteLink() = default;

        /**
         * @brief 非阻塞写出。
         * @return 实际被链路接收的字节数，0 表示链路忙或无接收者。
         * @note 返回值可能小于 size，调用方须自行续写，不得插入其他帧。
         */
        virtual int Write(const uint8_t* data, size_t size) = 0;

        /** @brief 注册接收回调，须在链路启动前调用。 */
        virtual void SetRxHandler(RxHandler handler, void* ctx) = 0;
    };

    /**
     * @brief bsp 链路公共部分：保存接收回调与最近一次收到的字节。
     * @details 子类构造时注册 Callback，收到字节后由本类派发给 Host。
     */
    class BspLink : public ByteLink {
      public:
        void SetRxHandler(RxHandler handler, void* ctx) override;

      protected:
        /** @brief 中断上下文入口，把最近一次收到的字节交给接收者。 */
        static void Callback(void* args);

        RxHandler handler_ = nullptr;
        void* ctx_ = nullptr;
        uint8_t* rx_data_ = nullptr;
        uint32_t rx_length_ = 0;
    };

    /** @brief 串口链路。 */
    class UartLink final : public BspLink {
      public:
        /** @param[in] uart 已完成 SetupRx/SetupTx 的串口实例。 */
        explicit UartLink(bsp::UART* uart);

        int Write(const uint8_t* data, size_t size) override;

      private:
        bsp::UART* uart_;
    };

#ifndef NO_USB
    /** @brief USB CDC 虚拟串口链路。 */
    class UsbLink final : public BspLink {
      public:
        /** @param[in] usb 已完成 SetupRx/SetupTx 的 USB 实例。 */
        explicit UsbLink(bsp::VirtualUSB* usb);

        int Write(const uint8_t* data, size_t size) override;

      private:
        bsp::VirtualUSB* usb_;
    };
#endif

}  // namespace host
