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

#include <stdint.h>

/*
 * 裁判系统 CRC 校验，实现与函数命名取自官方协议附录一「CRC 校验代码示例」。
 *
 * 2026 V1.3.0 与 2027 V1.0.0 的附录一逐字节相同：
 *   CRC8  初值 0xFF，多项式 G(x) = x^8 + x^5 + x^4 + 1
 *   CRC16 初值 0xFFFF，多项式 G(x) = x^16 + x^12 + x^5 + 1
 * 因此两赛季共用这一份实现，赛季切换不涉及本文件。
 *
 * 官方示例中的 Uint8_t / U8 / __FALSE 等笔误已按标准类型修正，
 * 算法与查表顺序保持原样。
 */

namespace referee_comm {

    /** @brief CRC8 初值。 */
    constexpr uint8_t CRC8_INIT = 0xff;
    /** @brief CRC16 初值。 */
    constexpr uint16_t CRC16_INIT = 0xffff;

    /**
     * @brief 计算 CRC8。
     * @param[in] pchMessage 待校验数据首地址。
     * @param[in] dwLength 参与计算的字节数。
     * @param[in] ucCRC8 初始值，帧头校验取 CRC8_INIT。
     * @return 计算得到的 CRC8。
     */
    uint8_t Get_CRC8_Check_Sum(const uint8_t* pchMessage, uint32_t dwLength, uint8_t ucCRC8);

    /**
     * @brief 校验 CRC8。
     * @param[in] pchMessage 数据首地址，末字节为待比对的 CRC8。
     * @param[in] dwLength 数据字节数加校验字节，即总长度。
     * @return 校验通过返回 1，否则返回 0。
     */
    uint32_t Verify_CRC8_Check_Sum(const uint8_t* pchMessage, uint32_t dwLength);

    /**
     * @brief 计算 CRC8 并写入末尾。
     * @param[in,out] pchMessage 数据首地址，末字节由本函数覆写。
     * @param[in] dwLength 数据字节数加校验字节，即总长度。
     * @note dwLength 须大于 2，否则直接返回。
     */
    void Append_CRC8_Check_Sum(uint8_t* pchMessage, uint32_t dwLength);

    /**
     * @brief 计算 CRC16。
     * @param[in] pchMessage 待校验数据首地址。
     * @param[in] dwLength 参与计算的字节数。
     * @param[in] wCRC 初始值，帧尾校验取 CRC16_INIT。
     * @return 计算得到的 CRC16。
     */
    uint16_t Get_CRC16_Check_Sum(const uint8_t* pchMessage, uint32_t dwLength, uint16_t wCRC);

    /**
     * @brief 校验 CRC16。
     * @param[in] pchMessage 数据首地址，末两字节为待比对的 CRC16，低字节在前。
     * @param[in] dwLength 数据字节数加校验字节，即总长度。
     * @return 校验通过返回 1，否则返回 0。
     */
    uint32_t Verify_CRC16_Check_Sum(const uint8_t* pchMessage, uint32_t dwLength);

    /**
     * @brief 计算 CRC16 并写入末尾两字节，低字节在前。
     * @param[in,out] pchMessage 数据首地址，末两字节由本函数覆写。
     * @param[in] dwLength 数据字节数加校验字节，即总长度。
     * @note dwLength 须大于 2，否则直接返回。
     */
    void Append_CRC16_Check_Sum(uint8_t* pchMessage, uint32_t dwLength);

}  // namespace referee_comm
