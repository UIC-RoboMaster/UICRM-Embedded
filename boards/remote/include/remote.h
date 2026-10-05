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
#include <cstddef>
#include <cstdint>
#include <cstring>

/** @brief 遥控协议解析与状态管理。 */
namespace remote {
    /**
     * @brief 遥控接收器的公共状态接口。
     * @details 仅管理有效帧序号、接收时间及协议失控标志，不统一各协议的通道布局。
     * UART、时钟和控制映射由接入层提供。校验失败或数据未收齐时不提交状态。
     * @note 所有方法须在同一线程调用或由接入层同步保护，不保证跨 ISR/任务的原子快照。
     */
    class RemoteReceiver {
      public:
        /**
         * @brief 设置接收器的初始在线超时。
         * @param[in] online_threshold_ms 超时阈值，单位毫秒，默认 250。
         */
        explicit RemoteReceiver(uint32_t online_threshold_ms = 250) : online_threshold_ms_(online_threshold_ms) {
        }

        /** @brief 支持通过基类指针销毁派生接收器。 */
        virtual ~RemoteReceiver() = default;

        /**
         * @brief 输入接收字节，具体边界和校验规则由派生协议定义。
         * @param[in] data 接收数据地址，调用结束后不保留该指针；空指针不提交有效帧。
         * @param[in] size 数据字节数。
         * @param[in] now_ms 本次输入的时间，单位毫秒，须与 IsOnline 使用相同时间基准。
         * @note DBUS 每次输入必须是具有明确边界的完整 18 字节块；SBUS/VT13 可输入半包或连续字节流。
         */
        virtual void UpdateData(const uint8_t* data, size_t size, uint32_t now_ms) = 0;

        /**
         * @brief 判断最近一次有效帧是否仍在超时范围内。
         * @param[in] now_ms 当前时间，单位毫秒，须与 UpdateData 使用相同时间基准。
         * @return 已接收有效帧且时间差严格小于阈值时为 true，否则为 false。
         * @note 支持时间为 0 的首帧及 uint32_t 计时回绕；超过完整计时周期无法区分。
         * 在线仅表示通信仍在更新，不代表 IsFailsafe 为 false 或机器人允许控制。
         */
        bool IsOnline(uint32_t now_ms) const {
            return received_ && uint32_t(now_ms - last_valid_ms_) < online_threshold_ms_;
        }

        /** @brief 获取有效帧提交序号，每帧递增一次，按 uint32_t 自然回绕；首次提交前为 0。 */
        uint32_t GetSequence() const {
            return sequence_;
        }

        /**
         * @brief 获取最近有效帧的协议失控标志。
         * @return 最近有效帧报告失控时为 true，否则为 false；首次有效帧前为 false。
         * @note 无失控标志的协议提交 false；此接口不包含超时或机器人模式判定。
         */
        bool IsFailsafe() const {
            return failsafe_;
        }

        /**
         * @brief 获取最近一次有效帧的提交时间，单位毫秒。
         * @return 最近有效帧的 now_ms；首次提交前为 0，是否已有帧须使用 HasState 判断。
         */
        uint32_t GetLastValidTimeMs() const {
            return last_valid_ms_;
        }

        /** @brief 判断是否曾接收有效帧；后续超时或重置半包不会清除此标志。 */
        bool HasState() const {
            return received_;
        }

        /**
         * @brief 修改在线超时阈值，不清除最后状态或接收时间。
         * @param[in] threshold_ms 超时阈值，单位毫秒；设为 0 时 IsOnline 始终返回 false。
         */
        void SetThreshold(uint32_t threshold_ms) {
            online_threshold_ms_ = threshold_ms;
        }

      protected:
        /**
         * @brief 派生接收器在完整校验并更新协议状态后提交有效帧。
         * @param[in] now_ms 有效帧的接收时间，单位毫秒。
         * @param[in] failsafe 当前帧的协议失控标志。
         * @details 更新接收时间及失控标志，标记已有状态，并将有效帧序号递增一次。
         */
        void CommitFrame(uint32_t now_ms, bool failsafe) {
            last_valid_ms_ = now_ms;
            failsafe_ = failsafe;
            received_ = true;
            ++sequence_;
        }

      private:
        uint32_t online_threshold_ms_;     ///< 在线超时阈值，单位毫秒。
        uint32_t last_valid_ms_ = 0;       ///< 最近有效帧时间，单位毫秒。
        uint32_t sequence_ = 0;           ///< 有效帧提交序号。
        bool received_ = false;          ///< 是否曾提交有效帧。
        bool failsafe_ = false;          ///< 最近有效帧的协议失控标志。
    };
    /** @brief 接收器内部字节解码工具，不涉及 UART、控制映射或有效帧提交。 */
    namespace detail {
        /**
         * @brief 按低位优先的位序提取无符号字段，可跨越字节边界。
         * @param[in] data 数据起始地址，不得为空。
         * @param[in] bit_offset 起始位偏移，单位为位；0 表示 data[0] 的最低位。
         * @param[in] bit_count 提取位数，范围 0～16；为 0 时返回 0。
         * @return 提取后的无符号数，源起始位成为结果最低位。
         * @pre data 必须覆盖全部待读位，bit_offset + bit_count 不得溢出。
         * @note 不检查缓冲长度，不做符号扩展、数值范围校验或中心转换。
         * 例如 DecodeBits(data, 11, 11) 提取 DBUS 通道 1 的原始编码。
         */
        inline uint16_t DecodeBits(const uint8_t* data, size_t bit_offset, unsigned bit_count) {
            uint16_t value = 0;
            for (unsigned i = 0; i < bit_count; ++i) {
                value |= uint16_t((data[(bit_offset + i) / 8] >> ((bit_offset + i) % 8)) & 1u) << i;
            }
            return value;
        }

        /**
         * @brief 从两个小端字节解码无符号 16 位整数。
         * @param[in] data 字段起始地址，须至少包含两个可读字节，不得为空。
         * @return data[0] 为低字节、data[1] 为高字节，结果范围 0～65535。
         * @note 不检查缓冲长度，与主机字节序及对齐方式无关。
         * 例如字节 {0x34, 0x12} 解码为 0x1234，可用于键盘位图和帧内 CRC 值。
         */
        inline uint16_t DecodeUint16LE(const uint8_t* data) {
            return uint16_t(data[0]) | (uint16_t(data[1]) << 8);
        }

        /**
         * @brief 从两个小端字节解码二进制补码形式的有符号 16 位整数。
         * @param[in] data 字段起始地址，须至少包含两个可读字节，不得为空。
         * @return 保留符号的数值，范围 -32768～32767。
         * @details 先读取无符号编码；编码超过 32767 时减去 65536 得到负值。
         * @note 不检查缓冲长度，不做死区或比例转换；例如 {0xFF, 0xFF} 解码为 -1。
         */
        inline int16_t DecodeInt16LE(const uint8_t* data) {
            const uint16_t value = DecodeUint16LE(data);
            return static_cast<int16_t>(value <= 32767 ? int32_t(value) : int32_t(value) - 65536);
        }

        /**
         * @brief 错帧后从缓存后缀查找下一个可能的帧头，并将保留数据移到缓存起始位置。
         * @tparam N 固定缓存容量，单位字节。
         * @param[in,out] buffer 当前候选帧缓存；保留的数据可能只是下一帧的半包。
         * @param[in,out] used 调用前为有效字节数，调用后为保留字节数。
         * @param[in] sof 要查找的帧头字节，例如 SBUS 的 0x0F 或 VT13 的 0xA9。
         * @pre 1 <= used <= N，只在当前候选帧被判定无效后调用。
         * @details 从下标 1 开始查找，跳过当前无效帧头；找到后保留该字节及其后的全部数据。
         * 未找到时 used 置 0。buffer 中 used 之后的内容无效，不必清零。
         * @note 只恢复候选边界，不验证帧长度、命令字或 CRC，不修改接收器状态。
         * DBUS 没有帧头，不能使用此方法恢复对齐。
         */
        template <size_t N>
        void FindNextFrame(std::array<uint8_t, N>& buffer, size_t& used, uint8_t sof) {
            size_t offset = 1;
            while (offset < used && buffer[offset] != sof) {
                ++offset;
            }
            used -= offset;
            std::memmove(buffer.data(), buffer.data() + offset, used);
        }
    }  // namespace detail
}  // namespace remote
