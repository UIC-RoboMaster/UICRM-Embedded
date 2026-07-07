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

/**
 * @file vofa.h
 * @brief VOFA+ JustFloat 波形调试驱动
 */

#pragma once

#include <cstdint>

#include "bsp_uart.h"

namespace driver {

    /**
     * @brief VOFA+ JustFloat 波形发送与指令接收
     * @details 通过 UART 向 VOFA+ 发送多通道 float 波形，并可选接收在线调参指令。
     *
     * TX 帧格式：`[float] × n + frame_tail`（JustFloat 协议）
     *
     * RX 指令格式：`变量名=数值#`，例如 `kp=1.5#`
     *
     * @note 需绑定 `bsp::UART`，通常为 `print_use_uart()` 创建的 `print_uart`
     * @note 勿与 `print()` / `clear_screen()` 混用同一 UART，否则会破坏二进制波形
     * @note 典型用法：初始化时 `Attach()` + `BindChannels()`，在任务中更新变量后调用 `Send()`
     */
    /**
     * @brief VOFA+ JustFloat waveform transmitter and command receiver
     * @details Sends multi-channel float waveforms to VOFA+ over UART, with optional
     *          online parameter assignment.
     *
     * TX frame format: `[float] × n + frame_tail` (JustFloat protocol)
     *
     * RX command format: `name=value#`, e.g. `kp=1.5#`
     *
     * @note Bind to `bsp::UART`, usually `print_uart` created by `print_use_uart()`
     * @note Do not mix with `print()` / `clear_screen()` on the same UART
     * @note Typical usage: `Attach()` + `BindChannels()` at init, then `Send()` after
     *       updating bound variables in a task
     */
    class Vofa {
      public:
        /** @brief 最大波形通道数 */
        /** @brief maximum number of waveform channels */
        static constexpr uint8_t MaxChannels = 24;

        /** @brief JustFloat 默认帧尾（正无穷，0x7f800000） */
        /** @brief default JustFloat frame tail (positive infinity, 0x7f800000) */
        static constexpr uint32_t DefaultFrameTail = 0x7f800000u;

        /** @brief RX 接收缓冲区大小（字节） */
        /** @brief RX receive buffer size in bytes */
        static constexpr uint16_t RxBufferSize = 128;

        /** @brief RX 变量名最大长度（字节） */
        /** @brief maximum RX variable name length in bytes */
        static constexpr uint16_t RxNameMaxLength = 100;

        /** @brief 默认构造，需后续调用 Attach() 绑定 UART */
        /** @brief default constructor; call Attach() later to bind UART */
        Vofa() = default;

        /**
         * @brief 构造并绑定 UART
         * @param uart 绑定的 UART 实例，通常为 `print_uart`
         * @param frame_tail JustFloat 帧尾，默认 `DefaultFrameTail`
         */
        /**
         * @brief construct and attach to a UART instance
         * @param uart UART instance to bind, usually `print_uart`
         * @param frame_tail JustFloat frame tail, defaults to `DefaultFrameTail`
         */
        explicit Vofa(bsp::UART* uart, uint32_t frame_tail = DefaultFrameTail);

        /**
         * @brief 绑定 UART 与帧尾
         * @param uart 绑定的 UART 实例
         * @param frame_tail JustFloat 帧尾
         */
        /**
         * @brief attach UART and frame tail
         * @param uart UART instance to bind
         * @param frame_tail JustFloat frame tail
         */
        void Attach(bsp::UART* uart, uint32_t frame_tail = DefaultFrameTail);

        /**
         * @brief 绑定单个波形通道
         * @param index 通道索引，范围 [0, MaxChannels)
         * @param data 指向 float 变量的指针；Send() 时读取其当前值
         * @note 若 index + 1 大于当前 ChannelCount()，会自动扩展通道数
         */
        /**
         * @brief bind a single waveform channel
         * @param index channel index in [0, MaxChannels)
         * @param data pointer to a float variable read on Send()
         * @note channel count expands automatically if index + 1 exceeds ChannelCount()
         */
        void BindChannel(uint8_t index, const float* data);

        /**
         * @brief 批量绑定波形通道
         * @param channels 指向各通道 float 指针的数组
         * @param count 通道数量，最大为 MaxChannels
         * @note 会清空原有绑定后再设置
         */
        /**
         * @brief bind multiple waveform channels at once
         * @param channels array of pointers to float variables
         * @param count number of channels, at most MaxChannels
         * @note clears previous bindings before applying new ones
         */
        void BindChannels(const float* const* channels, uint8_t count);

        /** @brief 清空所有已绑定的波形通道 */
        /** @brief clear all bound waveform channels */
        void ClearChannels();

        /**
         * @brief 获取当前绑定的通道数量
         * @return 通道数
         */
        /**
         * @brief get the number of bound channels
         * @return channel count
         */
        uint8_t ChannelCount() const;

        /**
         * @brief 打包 JustFloat 帧并通过 UART 发送
         * @note 发送前需已 Attach() 且至少绑定 1 个通道
         * @note 可在任意任务中调用，频率由调用方决定
         */
        /**
         * @brief pack a JustFloat frame and transmit over UART
         * @note requires Attach() and at least one bound channel
         * @note may be called from any task; send rate is caller-defined
         */
        void Send();

        /**
         * @brief 配置 RX 在线调参并注册 UART 接收回调
         * @param names 变量名列表，与 VOFA+ 发送的指令名对应
         * @param count 变量数量
         * @param buffer_size UART 接收缓冲区大小，默认 RxBufferSize
         * @note 内部调用 `SetupRx()`、`SetupRxData()`、`RegisterCallback()`
         * @note 解析结果通过 VariableIndex() / VariableValue() 读取
         */
        /**
         * @brief configure RX parameter assignment and register UART callback
         * @param names variable name list matching VOFA+ commands
         * @param count number of variable names
         * @param buffer_size UART RX buffer size, defaults to RxBufferSize
         * @note internally calls SetupRx(), SetupRxData(), RegisterCallback()
         * @note read parsed results via VariableIndex() / VariableValue()
         */
        void SetupRx(const char** names, uint8_t count, uint32_t buffer_size = RxBufferSize);

        /**
         * @brief 处理 UART 接收到的指令数据
         * @param data 接收缓冲区指针
         * @param length 有效数据长度
         * @note 通常由 SetupRx() 注册的回调自动调用；也可手动调用
         */
        /**
         * @brief process received command data from UART
         * @param data receive buffer pointer
         * @param length valid data length in bytes
         * @note usually invoked by callback registered in SetupRx(); may be called manually
         */
        void HandleRx(const uint8_t* data, uint16_t length);

        /**
         * @brief 获取最近一次 RX 解析到的变量索引
         * @return 变量在 SetupRx() 传入 names 中的索引；未匹配时为 -1
         */
        /**
         * @brief get index of the last parsed RX variable
         * @return index in names passed to SetupRx(); -1 if no match
         */
        int32_t VariableIndex() const;

        /**
         * @brief 获取最近一次 RX 解析到的变量数值
         * @return 解析得到的 float 值
         */
        /**
         * @brief get value of the last parsed RX variable
         * @return parsed float value
         */
        float VariableValue() const;

      private:
        static void UartCallback(void* ctx);

        void ParseRx(uint16_t length);
        uint8_t MatchVariableName(uint16_t length);
        void ParseVariableValue(uint16_t length, int value_offset);
        void PackFrame();

        bsp::UART* uart_ = nullptr;
        uint32_t frame_tail_ = DefaultFrameTail;

        uint8_t tx_buffer_[(MaxChannels + 1) * sizeof(float)];
        uint8_t rx_buffer_[RxBufferSize];

        const float* channels_[MaxChannels] = {};
        uint8_t channel_count_ = 0;

        const char** rx_names_ = nullptr;
        uint8_t rx_name_count_ = 0;

        int32_t variable_index_ = -1;
        float variable_value_ = 0.f;

        uint8_t* rx_ptr_ = nullptr;
        uint32_t rx_len_ = 0;
    };

}  // namespace driver
