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

#include <array>

#include "bsp_uart.h"
#include "controller.h"

namespace controller {
    /** @brief RefereeRC 遥控模式编码；是否允许控制由本车逻辑决定。 */
    enum class RefereeRcMode : uint8_t {
        C = 0,  ///< C 模式。
        N = 1,  ///< N 模式。
        S = 2   ///< S 模式。
    };

    /** @brief RefereeRC 有效帧快照，通道减中心并处理死区，不包含本车控制映射。 */
    struct RefereeRcFrame {
        /**
         * @brief 五通道减去 1024，包含 ±10 死区；不缩放或额外限幅。
         * @details 输出范围 -660～660，中心为 0；顺序为右 X/Y、左 Y/X、拨轮。
         */
        std::array<int16_t, 5> channels{};
        RefereeRcMode mode = RefereeRcMode::C;  ///< 模式选择，不自动转换为协议失控标志。
        bool pause = false;                     ///< 暂停按钮状态。
        bool left_button = false;               ///< 遥控器左侧按钮状态，与鼠标左键不同。
        bool right_button = false;              ///< 遥控器右侧按钮状态，与鼠标右键不同。
        bool trigger = false;                   ///< 扳机状态。
        MouseState mouse{};                     ///< 鼠标状态，包含左、右、中键。
        KeyboardState keyboard{};               ///< 键盘状态，支持 code 和 bit 按键访问。
        uint16_t crc16 = 0;                     ///< 已通过校验的帧内 CRC16 值。
    };

    /**
     * @brief RefereeRC 21 字节遥控接收器，支持 UART 自动接收和手动输入。
     * @details 验证 0xA9 帧头、0x53 命令字、既有 CRC16、合法模式 0/1/2、
     * 五通道范围 364～1684，以及各占两位的鼠标左/右/中键编码 0/1。
     * 仅有效帧更新数据和独立在线时间，N/S 模式及暂停按钮不自动视为协议失控。
     * @note UART 直连仅用于纯遥控字节流；混合 A5/A9 数据须先由外部链路分帧。
     * 并发读取不保证整帧一致性；UpdateData 与 ResetStream 须在同一接收上下文串行调用或停止接收后调用。
     */
    class RefereeRC final : public ControllerReceiver {
      public:
        static constexpr size_t FRAME_SIZE = 21;       ///< 单帧长度，单位字节。
        static constexpr uint8_t SOF = 0xA9;           ///< RefereeRC 帧头。
        static constexpr uint8_t CMD = 0x53;           ///< RefereeRC 命令字。
        static constexpr uint16_t CHANNEL_MIN = 364;   ///< 五通道原始值有效下限，包含边界。
        static constexpr uint16_t CHANNEL_MAX = 1684;  ///< 五通道原始值有效上限，包含边界。
        static constexpr uint16_t CHANNEL_CENTER = 1024;
        static constexpr int16_t CHANNEL_DEADZONE = 10;
        /**
         * @brief 创建手动输入模式接收器，不分配 UART。
         * @param[in] online_threshold_ms 有效遥控帧超时阈值，单位 ms。
         */
        explicit RefereeRC(uint32_t online_threshold_ms = 250);

        /**
         * @brief 创建并拥有 UART，自动注册回调及配置接收缓存。
         * @param[in] huart 已配置的 UART 句柄，不得为空或重复绑定。
         * @param[in] online_threshold_ms 有效遥控帧超时阈值，单位 ms。
         * @note 内部配置 921600 baud、8N1；引脚、DMA、中断由板级配置。
         * 绑定前不得启动该 UART 的收发；对象须覆盖接收中断使用期。
         */
        explicit RefereeRC(UART_HandleTypeDef* huart, uint32_t online_threshold_ms = 250);

        /** @brief 释放内部 UART；调用前须停止接收及相关中断。 */
        ~RefereeRC() override;

        /** @brief 禁止复制，避免重复拥有 UART 和回调上下文。 */
        RefereeRC(const RefereeRC&) = delete;
        /** @brief 禁止复制赋值。 */
        RefereeRC& operator=(const RefereeRC&) = delete;

        /** @brief 使用系统毫秒时钟判断有效遥控帧是否在线。 */
        bool IsOnline() const;
        /** @brief 保留手动输入使用的显式时间查询接口。 */
        using ControllerReceiver::IsOnline;

        /**
         * @brief 输入纯 RefereeRC 遥控字节流，支持半包、连续帧及错误后的后缀恢复。
         * @param[in] data 接收字节地址，空指针不提交状态，调用结束后不保留该指针。
         * @param[in] size 数据字节数。
         * @param[in] now_ms 本次输入的时间，单位毫秒；完成的有效帧使用此时间提交。
         * @warning 不直接输入混合 A5/A9 图传流，应由 Protocol::Receive 完成分帧后接入。
         */
        void UpdateData(const uint8_t* data, size_t size, uint32_t now_ms) override;

        /** @brief 清除暂存半包，保留最后有效状态、序号和接收时间。 */
        void ResetStream();

        /**
         * @brief 拷贝最后一帧有效状态。
         * @param[out] output 成功时接收完整快照；失败时保持原值。
         * @return 曾接收有效帧时为 true，否则为 false。
         * @note 离线后仍可读取最后状态；控制前须检查 IsOnline 及本车模式允许条件。
         */
        bool GetState(RefereeRcFrame& output) const;

      private:
        /**
         * @brief 读取 UART 接收块并直接输入解析器，读取错误时丢弃半包。
         * @param[in] context 注册回调时传入的有效 RefereeRC 实例指针。
         */
        static void UartCallback(void* context);

        /**
         * @brief 校验并解码内部缓存的完整候选帧，对有效数据更新快照并提交状态。
         * @param[in] now_ms 本帧接收时间，单位毫秒。
         * @return 校验通过并提交时为 true，否则为 false。
         * @details 校验帧头、命令字、CRC16、模式编码、通道范围及鼠标按钮；解码五通道、控制按钮和键鼠。
         * 所有校验通过后才整体替换最后状态并调用 CommitFrame；失败时状态、序号及有效时间不变。
         * @note 仅由内部完整候选帧入口调用，不负责收集半包或恢复字节边界。
         */
        bool Decode(uint32_t now_ms);
        bsp::UART* uart_ = nullptr;                 ///< 独占 UART；手动输入模式为 nullptr。
        std::array<uint8_t, FRAME_SIZE> buffer_{};  ///< 候选帧缓存。
        size_t used_ = 0;                           ///< 已缓存字节数。
        RefereeRcFrame state_{};                    ///< 最后一帧有效状态。
    };
}  // namespace controller
