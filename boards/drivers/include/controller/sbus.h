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
/**
 * @file sbus.h
 * @brief SBUS 协议接收器及 FS-i6X/iA10B 设备适配接口。
 */
#pragma once

#include <array>

#include "bsp_uart.h"
#include "controller.h"

namespace controller {
    /** @brief SBUS 原始帧快照，不做减中心、死区处理或设备映射。 */
    struct SbusFrame {
        std::array<uint16_t, 16> channels{};  ///< CH1～16 原始 11 位值，范围 0～2047。
        uint8_t flags = 0;                    ///< 原始标志字节，保留所有位。
        bool channel17 = false;               ///< flags bit0：数字通道 17。
        bool channel18 = false;               ///< flags bit1：数字通道 18。
        bool frame_lost = false;              ///< flags bit2：接收机报告丢帧，不等同于失控。
        bool failsafe = false;                ///< flags bit3：接收机报告失控，不等同于通信离线。
    };

    /**
     * @brief 通用 SBUS 接收器，管理 UART、分帧、原始状态及有效帧时间。
     * @details 可绑定 UART 自动接收，也可通过 UpdateData 手动输入字节流。
     * 每个有效帧更新状态和序号；帧中的失控标志不阻止在线时间更新。
     * @note 不提供内部同步；与接收中断并发读取时，不保证整帧一致性。
     * UpdateData、ResetStream、SetFooter 须在同一接收上下文串行调用或停止接收后调用。
     * @see I6x
     */
    class SBUS : public ControllerReceiver {
      public:
        static constexpr size_t FRAME_SIZE = 25;  ///< SBUS 单帧字节数。

        /**
         * @brief 创建手动输入模式接收器，不分配 UART。
         * @param[in] online_threshold_ms 有效帧超时阈值，单位 ms；0 表示始终离线。
         * @param[in] footer 唯一允许的帧尾字节，应与实际接收机一致。
         */
        explicit SBUS(uint32_t online_threshold_ms = 250, uint8_t footer = 0x00);

        /**
         * @brief 创建并拥有一个 UART，配置接收缓存并注册接收回调。
         * @param[in] huart 已完成板级配置的 UART 句柄，不得为空或重复绑定。
         * @param[in] online_threshold_ms 有效帧超时阈值，单位 ms。
         * @param[in] footer 唯一允许的帧尾字节。
         * @note 内部配置 100000 baud、8E2（HAL 使用 9 位字长以包含偶校验位）。
         * 引脚、DMA、中断及信号反相由板级配置；绑定前不得启动该 UART 的收发。
         * 对象生命周期须覆盖 UART 中断使用期。
         */
        explicit SBUS(UART_HandleTypeDef* huart, uint32_t online_threshold_ms = 250, uint8_t footer = 0x00);

        /**
         * @brief 释放内部 UART；手动输入模式无需释放外设。
         * @pre 调用方已停止接收及相关中断，不再触发本对象的回调。
         */
        ~SBUS() override;

        /** @brief 禁止复制，避免重复拥有 UART 和回调上下文。 */
        SBUS(const SBUS&) = delete;
        /** @brief 禁止复制赋值，保持 UART 所有权唯一。 */
        SBUS& operator=(const SBUS&) = delete;

        /**
         * @brief 使用系统毫秒时钟判断最近有效帧是否在线。
         * @return 已收到有效帧且未达到超时阈值时为 true，否则为 false。
         * @note 手动输入的时间须与系统时钟同源，否则使用带 now_ms 的重载。
         * 在线不代表无失控；控制前还须检查帧状态及应用策略。
         */
        bool IsOnline() const;
        /** @brief 保留基类的显式毫秒时钟查询接口。 */
        using ControllerReceiver::IsOnline;

        /**
         * @brief 拷贝最后一帧有效原始状态。
         * @param[out] output 成功时接收原始快照；失败时保持原值。
         * @return 曾收到有效帧时为 true，首次有效帧前为 false。
         * @note 离线后保留最后状态，返回值不表示输入当前可用于控制。
         */
        bool GetRawState(SbusFrame& output) const;

        /**
         * @brief 输入字节流，累积半包并逐帧校验、解码。
         * @param[in] data 输入数据，不保留指针；为空时忽略本次输入。
         * @param[in] size 输入字节数；0 表示不处理数据。
         * @param[in] now_ms 接收时间，单位 ms；本次完成的所有有效帧使用此时间。
         * @details 查找 0x0F 帧头，收齐 25 字节后检查配置的帧尾。
         * 无效候选帧保留下一帧头起始的后缀，继续处理后续字节。
         * 无效数据不刷新最后有效状态；有效失控帧仍更新状态及在线时间。
         * @note SBUS 无 CRC，帧头和帧尾校验不能保证数据完整性。
         */
        void UpdateData(const uint8_t* data, size_t size, uint32_t now_ms) override;

        /** @brief 丢弃暂存半包，保留最后有效帧、序号、时间及失控状态。 */
        void ResetStream();

        /**
         * @brief 修改唯一允许的帧尾字节，并丢弃暂存半包。
         * @param[in] footer 实际接收机使用的帧尾字节。
         * @note 不清除最后有效帧和接收状态。
         */
        void SetFooter(uint8_t footer);

      private:
        /**
         * @brief 将 UART 回调转发到接收器实例。
         * @param[in] context 注册回调时传入的有效 SBUS 实例指针。
         */
        static void UartCallback(void* context);

        /**
         * @brief 读取 UART 接收块，以系统毫秒时间调用 UpdateData。
         * @details 空接收块不处理；读取错误或非空块缺少数据指针时清除半包。
         */
        void RxCompleteCallback();

        /**
         * @brief 校验并解码完整候选帧，成功后发布原始状态。
         * @param[in] now_ms 本帧有效接收时间，单位 ms。
         * @return 帧头、帧尾正确且已提交状态时为 true，否则为 false。
         * @pre buffer_ 中已收齐 FRAME_SIZE 字节。
         * @note 失败不改变最后有效状态；本函数不负责重新查找帧头。
         */
        bool Decode(uint32_t now_ms);

        bsp::UART* uart_ = nullptr;                 ///< 独占的 UART；手动输入模式为 nullptr。
        std::array<uint8_t, FRAME_SIZE> buffer_{};  ///< 正在收集的候选帧。
        size_t used_ = 0;                           ///< 候选帧内已缓存的字节数。
        uint8_t footer_;                            ///< 当前允许的帧尾字节。
        SbusFrame state_{};                         ///< 最后有效帧，离线时仍保留。
    };

    /**
     * @brief i6X 控件快照，原始帧通过继承的 GetRawState 获取。
     * @details 丢帧及失控标志来自参与映射的同一帧，不包含在线超时判定。
     */
    struct I6xState {
        std::array<int16_t, 6> channels{};                 ///< CH1～6：右 X/Y、左 Y/X、VrA/VrB，范围 ±660。
        TwoPositionSwitch sw_a = TwoPositionSwitch::Down;  ///< CH7：SwA，两档。
        TwoPositionSwitch sw_b = TwoPositionSwitch::Down;  ///< CH8：SwB，两档。
        ThreePositionSwitch sw_c = ThreePositionSwitch::Mid;  ///< CH9：SwC，三档。
        TwoPositionSwitch sw_d = TwoPositionSwitch::Down;     ///< CH10：SwD，两档。
        bool frame_lost = false;                              ///< 接收机报告丢帧，原样传递。
        bool failsafe = false;                                ///< 接收机报告失控，原样传递。
    };

    /**
     * @brief FS-i6X/iA10B 设备适配，继承 SBUS 的接收和在线状态。
     * @details 使用 240/1024/1807 标定端点及中心，原始 ±10 死区包含边界。
     * 中心两侧按各自行程缩放并四舍五入，输出饱和至 ±660。
     * 低拨杆通道值对应物理上档，高值对应物理下档；模拟通道保持固定方向。
     * @note 发射机应保持 I6xState 所述通道分配；本类不包含机器人动作。
     * 不另建 UART、接收缓存或在线时间。
     */
    class I6x final : public SBUS {
      public:
        /** @brief 继承 SBUS 的手动输入及 UART 构造，使用固定设备映射。 */
        using SBUS::SBUS;

        /**
         * @brief 从基类最后有效帧转换得到 i6X 控件状态。
         * @param[out] output 成功时接收映射结果；失败时保持原值。
         * @return 曾收到有效帧时为 true，首次有效帧前为 false。
         * @note 不另外缓存映射结果，不修改接收时间和序号。
         * 离线或失控后仍可读取；控制前须另行判断输入是否可用。
         */
        bool GetState(I6xState& output) const;

      private:
        /**
         * @brief 对模拟通道执行减中心、死区、标定缩放和限幅。
         * @param[in] raw SBUS 原始通道值，范围 0～2047。
         * @return 范围为 -660～660 的映射值，死区内为 0。
         */
        static int16_t MapChannel(uint16_t raw);

        /**
         * @brief 以原始中心值 1024 为界解码两档拨杆。
         * @param[in] raw SBUS 原始通道值。
         * @return raw < 1024 为 Up，否则为 Down。
         */
        static TwoPositionSwitch TwoPosition(uint16_t raw);

        /**
         * @brief 按最近标定位置解码三档拨杆。
         * @param[in] raw SBUS 原始通道值。
         * @return 小于 632 为 Up，632～1415 为 Mid，至少 1416 为 Down。
         */
        static ThreePositionSwitch ThreePosition(uint16_t raw);
    };
}  // namespace controller
