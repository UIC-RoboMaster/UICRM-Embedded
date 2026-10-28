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

#include "controller.h"
#include "bsp_uart.h"

namespace controller {
    /**
     * @brief DBUS 有效帧快照，通道数值沿用旧 DBUS 的减中心与死区处理。
     * @details 原始值减去 1024，绝对值不大于 10 时置零，正常行程为 -660～660。
     * 不额外缩放或限幅；任何通道的原始值超出 364～1684 时拒绝整帧。
     */
    struct DbusFrame {
        /**
         * @brief 五个通道的有符号输入值，正常行程为 -660～660，中心为 0。
         * @details 下标 0：右摇杆 X；1：右摇杆 Y；2：左摇杆 X；3：左摇杆 Y；4：左拨轮。
         * 摇杆方向沿用旧代码：X 左负右正，Y 上负下正；拨轮方向沿用设备原编码。
         * @note 遥控器的样式
         *
         * ch4()
         *
         *  L_sw*                       *R_sw
         *       ch3-^            ^-ch1
         *    ch2-<     >+    -<     >+ch0
         *          +v            v+
         */
        std::array<int16_t, 5> channels{};
        ThreePositionSwitch right_switch = ThreePositionSwitch::Mid;  ///< 右拨杆位置。
        ThreePositionSwitch left_switch = ThreePositionSwitch::Mid;   ///< 左拨杆位置。
        MouseState mouse{};        ///< 鼠标状态；DBUS 不提供中键，middle 始终为 false。
        KeyboardState keyboard{};  ///< 键盘状态，支持 code 和 bit 按键访问。
    };

    /**
     * @brief DBUS 18 字节接收器，支持通过 UART 句柄构造后自动接收。
     * @details DBUS 无帧头及 CRC，校验长度、拨杆、鼠标按钮和配置的通道范围。
     * 字段合理性检查不能保证完整性，也不能用于猜测字节流的帧边界。
     * @note UART 模式内部通过 IDLE 回调接收；并发读取不保证整帧一致性。
     * UpdateData 与 SetChannelRange 不得并发调用，UART 模式下修改范围须先停止接收。
     */
    class DBUS final : public ControllerReceiver {
      public:
        static constexpr size_t FRAME_SIZE = 18;       ///< 单帧长度，单位字节。
        static constexpr uint16_t CHANNEL_MIN = 364;   ///< 原始通道有效下限。
        static constexpr uint16_t CHANNEL_MAX = 1684;  ///< 原始通道有效上限。

        /**
         * @brief 绑定并拥有 UART 接收对象，自动配置缓存和接收回调。
         * @param[in] huart 已完成板级配置的 UART 句柄，不得为同一 UART 重复创建接收对象。
         * @param[in] online_threshold_ms 在线超时，单位毫秒，默认 250。
         * @note 内部配置 100000 baud、8E1（HAL 使用 9 位字长以包含偶校验位）。
         * 引脚、DMA、中断及信号反相由板级配置；绑定前不得启动该 UART 的收发。
         * 对象须覆盖中断使用周期。
         */
        explicit DBUS(UART_HandleTypeDef* huart, uint32_t online_threshold_ms = 250);

        /** @brief 释放本接收器拥有的 UART；调用前须停止相关接收中断。 */
        ~DBUS() override;

        DBUS(const DBUS&) = delete;

        DBUS& operator=(const DBUS&) = delete;

        /** @brief 使用系统毫秒时钟判断有效 DBUS 帧是否在线，无需调用方传时间。 */
        bool IsOnline() const;

        /**
         * @brief 输入具有明确边界的完整 DBUS 接收块，一次调用只处理一帧。
         * @param[in] data 接收块地址，须从帧起点开始，调用结束后不保留该指针。
         * @param[in] size 接收块字节数，必须恰好为 FRAME_SIZE（18 字节）。
         * @param[in] now_ms 本块接收时间，单位毫秒。
         * @details UART IDLE 回调自动调用本接口；手动接入时调用方负责提供帧边界。
         * 空指针、错误长度或无效字段不修改状态、序号或有效帧时间。
         * @note 不累积半包，不拆分多帧块；5+13 字节的两次调用和一次 36 字节输入均拒绝。
         */
        void UpdateData(const uint8_t* data, size_t size, uint32_t now_ms) override;

        /**
         * @brief 拷贝最后一帧有效状态。
         * @param[out] output 成功时接收完整快照；失败时保持原值。
         * @return 曾接收有效帧时为 true，否则为 false。
         * @note 离线后仍可读取最后状态；用于控制前须另行判断 IsOnline。
         */
        bool GetState(DbusFrame& output) const;

        /**
         * @brief 设置五个通道统一使用的原始值校验范围，默认 364～1684。
         * @param[in] minimum 允许的最小值，包含边界。
         * @param[in] maximum 允许的最大值，包含边界。
         * @return 364 <= minimum <= maximum <= 1684 时为 true；否则返回 false 并保留配置。
         * @note 参数使用线上的原始编码值，仅影响后续帧校验；输出仍按旧 DBUS 语义减中心并处理死区。
         */
        bool SetChannelRange(uint16_t minimum, uint16_t maximum);

      private:
        /** @brief UART 接收回调入口，读取接收块并按 IDLE 边界校验。 */
        void RxCompleteCallback();

        /**
         * @brief 将 DBUS 两位拨杆编码显式转换为公共三档位置。
         * @param[in] raw 协议编码：1 为上档，2 为下档，3 为中档，0 非法。
         * @param[out] output 成功时接收逻辑位置；失败时保持原值。
         * @return 编码合法时为 true，否则为 false。
         */
        static bool DecodeSwitch(uint16_t raw, ThreePositionSwitch& output);

        /**
         * @brief 校验并解码一帧，对有效数据更新快照并提交状态。
         * @param[in] data 已收齐的 FRAME_SIZE 字节候选帧。
         * @param[in] now_ms 本帧接收时间，单位毫秒。
         * @return 校验通过并提交时为 true，否则为 false。
         * @details 校验五通道原始范围、左右拨杆及鼠标按钮；转换通道中心与死区，解码键鼠。
         * 所有校验通过后才整体替换最后状态并调用 CommitFrame；失败时状态、序号及有效时间不变。
         * @note 仅由内部完整候选帧入口调用，不负责收集半包或恢复字节边界。
         */
        bool Decode(const uint8_t* data, uint32_t now_ms);
        static void UartCallback(void* context);  ///< UART 回调转发入口。
        bsp::UART* uart_ = nullptr;               ///< 内部独占的 UART。
        uint16_t channel_min_ = CHANNEL_MIN;      ///< 原始通道值校验下限。
        uint16_t channel_max_ = CHANNEL_MAX;      ///< 原始通道值校验上限。
        DbusFrame state_{};                       ///< 最后一帧有效状态。
    };
}  // namespace controller
