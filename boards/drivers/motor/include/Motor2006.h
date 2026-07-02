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

#pragma once

#include "DjiMotorBase.h"

namespace driver {

/**
 * @brief DJI M2006/P36 减速电机配置
 *
 * C610 电调: raw_current ∈ [-10000, 10000] 对应转矩电流 ∈ [-3A, 3A]
 * 编码器: 8192 线/圈，增量式
 * 减速比: 36:1
 *
 * RX_ID  = 0x200 + 电机 ID
 * TX_ID 自动识别: 0x201~0x204 → 0x200, 0x205~0x208 → 0x1ff
 */
struct Motor2006Config {
    static constexpr int16_t MAX_OUTPUT_CURRENT = 10000;          ///< 最大输出电流 [raw], ~10A
    static constexpr float CURRENT_TO_AMP = 3.0f / 16384.0f;     ///< 原始电流 → 安培 换算系数
    static constexpr float ENCODER_RESOLUTION = 8192.0f;          ///< 编码器分辨率 [线/圈]
    static constexpr float RATED_TORQUE_CONSTANT = 180.0f;        ///< 额定转矩常数 [mN·m/A]
    static constexpr float ORIGINAL_TRANSMISSION_RATIO = 36.0f;   ///< 减速比 (原始)
};

/**
 * @brief DJI M2006/P36 减速电机
 *
 * 搭配 C610 电调使用，支持角度/速度/力矩三种控制模式。
 * 通过 36:1 减速箱驱动输出轴，适用于 RoboMaster 拨弹机构等功能部件。
 */
class Motor2006 : public DjiMotorBase {
  public:
    /**
     * @brief M2006 构造函数
     * @param can    CAN 对象
     * @param rx_id  RX ID = 0x200 + 电机 ID
     * @param tx_id  电机接收报文标识符，0x00 表示自动解析
     */
    Motor2006(bsp::CAN* can, uint16_t rx_id, uint16_t tx_id = 0x00);

    /**
     * @brief CAN 数据更新回调
     * @note 由 CAN 接收中断调用，不应在其他上下文手动调用
     * @param data 8 字节 CAN 数据帧 [Theta_H, Theta_L, Omega_H, Omega_L, Curr_H, Curr_L, _, _]
     */
    void UpdateData(const uint8_t data[]) override final;

    /**
     * @brief 打印电机调试数据
     * @note 输出在线状态、角度、角速度、原始电流值
     */
    void PrintData() const override final;

    /**
     * @brief 设置电机输出电流
     * @note 自动钳位到 [-MAX_OUTPUT_CURRENT, MAX_OUTPUT_CURRENT]
     * @param val 原始电流值 [raw], raw_current ∈ [-16384, 16384] 对应转矩电流 ∈ [-3A, 3A]
     */
    void SetOutput(int16_t val) override final;

  private:
    /**
     * @brief 由 RX_ID 自动解析 TX_ID
     * @note M2006 + C610 电调标准 CAN 协议标识符
     * @param rx_id  RX ID = 0x200 + 电机 ID
     * @return tx_id  0x201~0x204 → 0x200, 0x205~0x208 → 0x1ff
     */
    static constexpr uint16_t ResolveTxId(uint16_t rx_id) {
        return (rx_id >= 0x205) ? 0x1ff : 0x200;
    }

    static const int16_t MAX_OUT = 10000;  ///< 最大输出电流 (未使用，见 Motor2006Config)
};

}  // namespace driver
