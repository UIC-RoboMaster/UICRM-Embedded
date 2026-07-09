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
 * @brief DJI M2006/P36 减速电机默认配置
 * @details 本结构体包含了电机编码器、电调电流映射、减速比以及转矩常数等核心物理参数
 * - **电调映射**：-10000 ~ 10000 对应 -10A ~ 10A
 * - **编码器**：0 ~ 8191 对应 0 ~ 360°
 * - **减速比**：36:1
 */
struct Motor2006Config {
    static constexpr int16_t MAX_RAW_THETA = 8191;           ///< 转子机械角最大值 [raw] 0->8191 对应 0~360°
    static constexpr int16_t MAX_RAW_CURRENT = 10000;        ///< 转矩电流反馈最大值（对应实际电流 -10A ~ 10A）
    static constexpr float MAX_CURRENT = 10.0f;              ///< 最大转矩电流 [A]
    static constexpr float RATED_TORQUE_CONSTANT = 0.18f;     ///< 额定转矩常数 [mN·m/A]
    static constexpr float ORIGINAL_TRANSMISSION_RATIO = 36.0f; ///< 默认减速比 (36:1)
};

/**
 * @class DJI M2006/P36 减速电机类
 * @brief 实现 DJI M2006/P36 减速电机的基本功能
 * @details 本类继承自 DjiMotorBase，实现了 DJI M2006/P36 减速电机的基本功能
 * - **构造函数**：初始化 CAN 通信和电机状态
 * - **CAN 数据更新回调**：由 CAN 接收中断调用，不应在其他上下文手动调用
 * - **打印电机调试数据**：输出在线状态、角度、角速度、原始电流值
 * - **设置电机输出电流**：自动钳位到 [-MAX_RAW_CURRENT, MAX_RAW_CURRENT]
 */
class Motor2006 : public DjiMotorBase {
  public:
    /**
     * @brief M2006 构造函数
     * @param can    硬件 CAN 对象
     * @param rx_id  RX ID = 0x200 + 电调 ID
     * @param tx_id  电调接收报文标识符，0x00 表示自动解析
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
     * @note 自动钳位到 [-MAX_RAW_CURRENT, MAX_RAW_CURRENT]
     * @param val 原始电流值 [raw], raw_current ∈ [-10000, 10000] 对应转矩电流 ∈ [-10A, 10A]
     */
    void SetOutput(int16_t val) override final;

  private:
    /**
     * @brief CAN 接收回调，转发至 Motor2006::UpdateData
     * @param ctx  指向 Motor2006 实例的指针
     * @param data 原始 CAN 数据
     */
    static void RxThunk(void* ctx, const uint8_t data[]);

    /**
     * @brief 由 RX_ID 自动解析 TX_ID
     * @note M2006 + C610 电调标准 CAN 协议标识符
     * @param rx_id  RX ID = 0x200 + 电调 ID
     * @return tx_id  由 rx_id 决定: 0x201 - 0x204 → 0x200, 0x205 - 0x208 → 0x1ff
     */
    static constexpr uint16_t ResolveTxId(uint16_t rx_id) {
        return (rx_id >= 0x205) ? 0x1ff : 0x200;
    }

    static const int16_t MAX_OUT = 10000;  ///< 最大输出电流 (未使用，见 Motor2006Config)
};

}  // namespace driver
