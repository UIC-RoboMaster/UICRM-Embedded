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
#include "remote.h"

namespace remote {
    /** @brief SBUS 有效帧快照，保留原始通道和协议标志，不做本车控制映射。 */
    struct SbusFrame {
        std::array<uint16_t, 16> channels{};  ///< 原始 11 位值 0～2047，下标 0 对应通道 1；部件映射由设备配置决定。
        uint8_t flags = 0;                   ///< 原始标志字节。
        bool channel17 = false;             ///< 数字通道 17。
        bool channel18 = false;             ///< 数字通道 18。
        bool frame_lost = false;            ///< 接收机报告丢帧，不自动等同于失控。
        bool failsafe = false;              ///< 接收机报告失控；有效失控帧仍会刷新通信在线。
    };

    /**
     * @brief SBUS 25 字节流接收器，支持通过 UART 句柄构造后自动接收。
     * @details 按 0x0F 帧头及配置的尾字节识别候选帧，支持半包、连续帧与错误后的后缀恢复。
     * SBUS 无 CRC，帧头和尾字节检查不能保证数据完整性。通道不减中心、不做死区或限幅。
     * @note 不绑定设备的摇杆、拨杆或通道端点；所有调用与状态读取须由接入层同步。
     */
    class SbusReceiver final : public RemoteReceiver {
      public:
        static constexpr size_t FRAME_SIZE = 25;  ///< 单帧长度，单位字节。
        /**
         * @brief 创建手动输入模式接收器，设置在线超时及允许的帧尾字节，不创建 UART。
         * @param[in] online_threshold_ms 在线超时，单位毫秒，默认 250。
         * @param[in] footer 唯一允许的帧尾字节，默认 0x00，须按实际接收机配置。
         */
        explicit SbusReceiver(uint32_t online_threshold_ms = 250, uint8_t footer = 0x00);
        /**
         * @brief 创建并拥有 UART 接收对象，自动配置缓存和注册接收回调。
         * @param[in] huart 已完成板级配置的 UART 句柄，不得重复绑定同一 UART。
         * @param[in] online_threshold_ms 在线超时，单位毫秒，默认 250。
         * @param[in] footer 唯一允许的帧尾字节，默认 0x00。
         * @note 波特率、数据位、校验位、停止位及反相由板级配置；对象须覆盖中断使用周期。
         */
        explicit SbusReceiver(UART_HandleTypeDef* huart, uint32_t online_threshold_ms = 250,
                              uint8_t footer = 0x00);
        /**
         * @brief 借用旧兼容包装的 UART，不创建第二个 UART、不替换原回调。
         * @param[in] uart 生命周期须长于本接收器的 UART 对象。
         * @param[in] online_threshold_ms 在线超时，单位毫秒，默认 250。
         * @param[in] footer 唯一允许的帧尾字节，默认 0x00。
         * @note 包装在自身回调中调用 RxCompleteCallback，接收器内部完成数据读取。
         */
        explicit SbusReceiver(bsp::UART& uart, uint32_t online_threshold_ms = 250,
                              uint8_t footer = 0x00);
        /** @brief 释放本接收器拥有的 UART；借用的 UART 不释放。 */
        ~SbusReceiver() override;
        SbusReceiver(const SbusReceiver&) = delete;
        SbusReceiver& operator=(const SbusReceiver&) = delete;
        /** @brief 使用系统毫秒时钟判断最近有效帧是否在线；失控标志须另行查询。 */
        bool IsOnline() const;
        /** @brief 保留显式时钟查询，供手动输入和测试使用。 */
        using RemoteReceiver::IsOnline;
        /**
         * @brief UART 回调入口，读取接收块并交给流解析；UART 模式自动调用。
         * @note IDLE 块可以是半包或连续数据，不要求等于一帧；读取失败时清除半包。
         */
        void RxCompleteCallback();
        /**
         * @brief 输入 SBUS 字节流，内部暂存半包并提交完整有效帧。
         * @param[in] data 接收字节地址，空指针不提交状态，调用结束后不保留该指针。
         * @param[in] size 数据字节数。
         * @param[in] now_ms 本次输入的时间，单位毫秒；完成的有效帧使用此时间提交。
         * @note 有效失控帧仍更新状态与在线时间，控制前须同时检查 IsFailsafe。
         */
        void UpdateData(const uint8_t* data, size_t size, uint32_t now_ms) override;
        /** @brief 清除暂存半包，保留最后有效状态、序号和接收时间。 */
        void ResetStream();
        /**
         * @brief 拷贝最后一帧有效状态。
         * @param[out] output 成功时接收完整快照；失败时保持原值。
         * @return 曾接收有效帧时为 true，否则为 false。
         * @note 离线后仍可读取最后状态；控制前须检查 IsOnline 和 IsFailsafe。
         */
        bool GetState(SbusFrame& output) const;
        /**
         * @brief 修改唯一允许的帧尾字节，并清除暂存半包。
         * @param[in] footer 实际接收机使用的帧尾字节。
         * @note 不清除最后有效状态、序号或接收时间；不会自动接受其他尾字节变体。
         */
        void SetFooter(uint8_t footer);

      private:
        /**
         * @brief 校验并解码内部缓存的完整候选帧，对有效数据更新快照并提交状态。
         * @param[in] now_ms 本帧接收时间，单位毫秒。
         * @return 校验通过并提交时为 true，否则为 false。
         * @details 校验帧头与配置的尾字节；解码 16 个原始通道及数字通道、丢帧和失控标志。
         * 所有校验通过后才整体替换最后状态并调用 CommitFrame；失败时状态、序号及有效时间不变。
         * @note 仅由内部完整候选帧入口调用，不负责收集半包或恢复字节边界。
         */
        bool Decode(uint32_t now_ms);
        static void UartCallback(void* context);  ///< UART 回调转发入口。
        bsp::UART* uart_ = nullptr;               ///< 内部拥有或旧包装借用的 UART。
        bool owns_uart_ = false;                 ///< 是否负责销毁 UART。
        std::array<uint8_t, FRAME_SIZE> buffer_{};  ///< 候选帧缓存。
        size_t used_ = 0;                          ///< 已缓存字节数。
        uint8_t footer_;                          ///< 唯一允许的帧尾字节。
        SbusFrame state_{};                       ///< 最后一帧有效状态。
    };
}  // namespace remote
