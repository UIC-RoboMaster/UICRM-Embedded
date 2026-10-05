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
#include "bsp_uart.h"

namespace remote {
    /** @brief DBUS 拨杆位置，数值与协议编码一致。 */
    enum class DbusSwitchPosition : uint8_t {
        Up = 1,      ///< 上
        Down = 2,    ///< 下
        Middle = 3   ///< 中
    };

    /**
     * @brief DBUS 鼠标输入，保持协议中的原始有符号数值。
     * @details 三轴范围均为 -32768～32767，静止值为 0，不套用摇杆的 ±660 范围或死区。
     * 原始左右键字段必须为 0 或 1，否则拒绝整帧；输出 false 表示松开，true 表示按下。
     */
    struct DbusMouseState {
        int16_t x = 0;        ///< 鼠标 X 轴输入，范围 -32768～32767。
        int16_t y = 0;        ///< 鼠标 Y 轴输入，范围 -32768～32767。
        int16_t z = 0;        ///< 鼠标滚轮输入，范围 -32768～32767。
        bool left = false;   ///< 鼠标左键是否按下。
        bool right = false;  ///< 鼠标右键是否按下。
    };

    /**
     * @brief DBUS 键盘状态，按旧 API 提供完整位图 code 和按键位域 bit。
     * @details 位序沿用旧代码；多个按键可同时为 1，不采用旧版手册中不同的键位定义。
     * code 与 bit 共享存储，支持 keyboard.code 或 keyboard.bit.W 等访问。
     * code 范围为 0～65535，每个按键位只能为 0 或 1，多个按键可同时按下。
     * @note 位域视图采用本项目 GCC/ARM 布局；线上数据仍按小端字节显式解码，
     * 不把接收缓存强制转换为该类型。新编译器平台须验证位域布局。
     */
    union DbusKeyboardState {
        uint16_t code = 0;  ///< 完整 16 位按键位图，默认没有按键按下。
        struct {
            uint16_t W : 1;  ///< bit0：W 键是否按下。
            uint16_t S : 1;  ///< bit1：S 键是否按下。
            uint16_t A : 1;  ///< bit2：A 键是否按下。
            uint16_t D : 1;  ///< bit3：D 键是否按下。
            uint16_t SHIFT : 1;  ///< bit4：SHIFT 键是否按下。
            uint16_t CTRL : 1;  ///< bit5：CTRL 键是否按下。
            uint16_t Q : 1;  ///< bit6：Q 键是否按下。
            uint16_t E : 1;  ///< bit7：E 键是否按下。
            uint16_t R : 1;  ///< bit8：R 键是否按下。
            uint16_t F : 1;  ///< bit9：F 键是否按下。
            uint16_t G : 1;  ///< bit10：G 键是否按下。
            uint16_t Z : 1;  ///< bit11：Z 键是否按下。
            uint16_t X : 1;  ///< bit12：X 键是否按下。
            uint16_t C : 1;  ///< bit13：C 键是否按下。
            uint16_t V : 1;  ///< bit14：V 键是否按下。
            uint16_t B : 1;  ///< bit15：B 键是否按下。
        } __attribute__((packed)) bit;
    } __attribute__((packed));
    static_assert(sizeof(DbusKeyboardState) == 2, "DBUS keyboard must occupy two bytes");

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
        DbusSwitchPosition right_switch = DbusSwitchPosition::Middle;  ///< 右拨杆位置。
        DbusSwitchPosition left_switch = DbusSwitchPosition::Middle;   ///< 左拨杆位置。
        DbusMouseState mouse{};             ///< 鼠标状态。
        DbusKeyboardState keyboard{};     ///< 键盘状态，支持 code 和 bit 按键访问。
    };

    /**
     * @brief DBUS 18 字节接收器，支持通过 UART 句柄构造后自动接收。
     * @details DBUS 无帧头及 CRC，校验长度、拨杆、鼠标按钮和配置的通道范围。
     * 字段合理性检查不能保证完整性，也不能用于猜测字节流的帧边界。
     * @note UART 模式内部通过 IDLE 回调接收；跨 ISR/任务读取仍须同步保护。
     */
    class DbusReceiver final : public RemoteReceiver {
      public:
        static constexpr size_t FRAME_SIZE = 18;  ///< 单帧长度，单位字节。
        static constexpr uint16_t CHANNEL_MIN = 364;   ///< 原始通道有效下限。
        static constexpr uint16_t CHANNEL_MAX = 1684;  ///< 原始通道有效上限。

        /**
         * @brief 绑定并拥有 UART 接收对象，自动配置缓存和接收回调。
         * @param[in] huart 已完成板级配置的 UART 句柄，不得为同一 UART 重复创建接收对象。
         * @param[in] online_threshold_ms 在线超时，单位毫秒，默认 250。
         * @note 波特率、偶校验、停止位及反相仍由板级配置负责；对象须覆盖中断使用周期。
         */
        explicit DbusReceiver(UART_HandleTypeDef* huart, uint32_t online_threshold_ms = 250);

        /**
         * @brief 复用旧包装已拥有的 UART，不创建第二个 UART 对象。
         * @param[in] uart 已存在的 UART，生命周期须长于本接收器。
         * @note 兼容包装在自身回调中调用 RxCompleteCallback；本构造不替换原回调。
         */
        explicit DbusReceiver(bsp::UART& uart, uint32_t online_threshold_ms = 250);

        /** @brief 释放本接收器拥有的 UART；借用的 UART 不释放。 */
        ~DbusReceiver() override;

        DbusReceiver(const DbusReceiver&) = delete;

        DbusReceiver& operator=(const DbusReceiver&) = delete;

        /** @brief 使用系统毫秒时钟判断有效 DBUS 帧是否在线，无需调用方传时间。 */
        bool IsOnline() const;

        /** @brief UART 接收回调入口，读取接收块并按 IDLE 边界校验；UART 模式自动调用。 */
        void RxCompleteCallback();

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
        /**
         * @brief 校验并解码一帧，对有效数据更新快照并提交状态。
         * @param[in] data 已收齐的 FRAME_SIZE 字节候选帧。
         * @param[in] now_ms 本帧接收时间，单位毫秒。
         * @return 校验通过并提交时为 true，否则为 false。
         * @details 校验五通道原始范围、左右拨杆及鼠标按钮；转换通道中心与死区，解码键鼠。
         * 所有校验通过后才整体替换最后状态并调用 CommitValidFrame；失败时状态、序号及有效时间不变。
         * @note 仅由内部完整候选帧入口调用，不负责收集半包或恢复字节边界。
         */
        bool Decode(const uint8_t* data, uint32_t now_ms);
        static void UartCallback(void* context);  ///< UART 回调转发入口。
        bsp::UART* uart_ = nullptr;              ///< 内部拥有或旧包装借用的 UART。
        bool owns_uart_ = false;                ///< 是否负责销毁 UART。
        uint16_t channel_min_ = CHANNEL_MIN;       ///< 原始通道值校验下限。
        uint16_t channel_max_ = CHANNEL_MAX;       ///< 原始通道值校验上限。
        DbusFrame state_{};                        ///< 最后一帧有效状态。
    };
}  // namespace remote
