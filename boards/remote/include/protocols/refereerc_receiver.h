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

#include "remote.h"

namespace remote {
    /** @brief VT13 模式编码；是否允许控制由本车逻辑决定。 */
    enum class Vt13Mode : uint8_t {
        C = 0,  ///< C 模式。
        N = 1,  ///< N 模式。
        S = 2   ///< S 模式。
    };

    /** @brief VT13 鼠标输入，保留协议中的原始有符号数值。 */
    struct Vt13MouseState {
        int16_t x = 0;         ///< 鼠标 X 轴输入。
        int16_t y = 0;         ///< 鼠标 Y 轴输入。
        int16_t z = 0;         ///< 鼠标滚轮输入。
        bool left = false;    ///< 鼠标左键是否按下。
        bool right = false;   ///< 鼠标右键是否按下。
        bool middle = false;  ///< 鼠标中键是否按下。
    };

    /**
     * @brief VT13 键盘状态，提供完整位图 code 和命名按键位域 bit。
     * @details 位序与 VT13 协议及旧 keyboard_t 一致；多个按键可同时按下。
     * code 与 bit 共享两字节存储，code 默认为 0，各按键位 0 表示松开、1 表示按下。
     * @note 位域视图采用本项目 GCC/ARM 布局；线上数据仍按小端字节显式解码，
     * 不将接收缓存强制转换为此类型。更换编译器平台须验证位域布局。
     */
    union Vt13KeyboardState {
        uint16_t code = 0;  ///< 完整 16 位按键位图，范围 0～65535。
        struct {
            uint16_t W : 1;      ///< bit0：W 键。
            uint16_t S : 1;      ///< bit1：S 键。
            uint16_t A : 1;      ///< bit2：A 键。
            uint16_t D : 1;      ///< bit3：D 键。
            uint16_t SHIFT : 1;  ///< bit4：SHIFT 键。
            uint16_t CTRL : 1;   ///< bit5：CTRL 键。
            uint16_t Q : 1;      ///< bit6：Q 键。
            uint16_t E : 1;      ///< bit7：E 键。
            uint16_t R : 1;      ///< bit8：R 键。
            uint16_t F : 1;      ///< bit9：F 键。
            uint16_t G : 1;      ///< bit10：G 键。
            uint16_t Z : 1;      ///< bit11：Z 键。
            uint16_t X : 1;      ///< bit12：X 键。
            uint16_t C : 1;      ///< bit13：C 键。
            uint16_t V : 1;      ///< bit14：V 键。
            uint16_t B : 1;      ///< bit15：B 键。
        } __attribute__((packed)) bit;
    } __attribute__((packed));
    static_assert(sizeof(Vt13KeyboardState) == 2, "VT13 keyboard must occupy two bytes");

    /** @brief VT13 有效帧快照，通道减中心并处理死区，不包含本车控制映射。 */
    struct Vt13Frame {
        /**
         * @brief 五通道减去 1024，包含 ±10 死区；不缩放或额外限幅。
         * @details 输出范围 -660～660，中心为 0；顺序为右 X/Y、左 Y/X、拨轮。
         */
        std::array<int16_t, 5> channels{};
        Vt13Mode mode = Vt13Mode::C;        ///< 模式选择，不自动转换为协议失控标志。
        bool pause = false;                ///< 暂停按钮状态。
        bool left_button = false;          ///< 遥控器左侧按钮状态，与鼠标左键不同。
        bool right_button = false;         ///< 遥控器右侧按钮状态，与鼠标右键不同。
        bool trigger = false;              ///< 扳机状态。
        Vt13MouseState mouse{};            ///< 鼠标状态。
        Vt13KeyboardState keyboard{};     ///< 键盘状态，支持 code 和 bit 按键访问。
        uint16_t crc16 = 0;               ///< 已通过校验的帧内 CRC16 值。
    };

    /**
     * @brief VT13 21 字节遥控协议接收器，无 UART、HAL 或系统时钟依赖。
     * @details 验证 0xA9 帧头、0x53 命令字、既有 CRC16、合法模式 0/1/2、
     * 五通道范围 364～1684，以及各占两位的鼠标左/右/中键编码 0/1。
     * 仅有效帧更新数据和独立在线时间，N/S 模式及暂停按钮不自动视为协议失控。
     * @note VT13 由图传链路承载；A5/A9 混合分帧位于 protocol.cpp，不属于本接收器职责。
     * 所有输入与读取须由接入层同步保护。
     */
    class RefereeRcReceiver final : public RemoteReceiver {
      public:
        static constexpr size_t FRAME_SIZE = 21;  ///< 单帧长度，单位字节。
        static constexpr uint8_t SOF = 0xA9;      ///< VT13 帧头。
        static constexpr uint8_t CMD = 0x53;      ///< VT13 命令字。
        static constexpr uint16_t CHANNEL_MIN = 364;   ///< 五通道原始值有效下限，包含边界。
        static constexpr uint16_t CHANNEL_MAX = 1684;  ///< 五通道原始值有效上限，包含边界。
        static constexpr uint16_t CHANNEL_CENTER = 1024;
        static constexpr int16_t CHANNEL_DEADZONE = 10;
        /** @brief 继承基类构造，可传入在线超时毫秒数，默认 250。 */
        using RemoteReceiver::RemoteReceiver;

        /**
         * @brief 输入纯 VT13 字节流，支持半包、连续帧及错误后的后缀恢复。
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
        bool GetState(Vt13Frame& output) const;

      private:
        /**
         * @brief 校验并解码内部缓存的完整候选帧，对有效数据更新快照并提交状态。
         * @param[in] now_ms 本帧接收时间，单位毫秒。
         * @return 校验通过并提交时为 true，否则为 false。
         * @details 校验帧头、命令字、CRC16、模式编码、通道范围及鼠标按钮；解码五通道、控制按钮和键鼠。
         * 所有校验通过后才整体替换最后状态并调用 CommitFrame；失败时状态、序号及有效时间不变。
         * @note 仅由内部完整候选帧入口调用，不负责收集半包或恢复字节边界。
         */
        bool Decode(uint32_t now_ms);
        std::array<uint8_t, FRAME_SIZE> buffer_{};  ///< 候选帧缓存。
        size_t used_ = 0;                          ///< 已缓存字节数。
        Vt13Frame state_{};                        ///< 最后一帧有效状态。
    };
}  // namespace remote
