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
 * @brief DJI GM6020 云台电机默认配置
 * @details 本结构体包含了电机编码器、电调电流映射、减速比以及转矩常数等核心物理参数
 * - **电调映射**：-16384 ~ 16384 对应 -3A ~ 3A
 * - **编码器**：0 ~ 8191 对应 0 ~ 360°
 * - **减速比**：1:1
 */
struct Motor6020Config {
    static constexpr int16_t MAX_RAW_THETA = 8191;           ///< 转子机械角最大值 [raw] 0->8191 对应 0~360°
    static constexpr int16_t MAX_RAW_CURRENT = 16384;         ///< 转矩电流反馈最大值 [raw] -16384->16384 对应 -3A~3A
    static constexpr float MAX_CURRENT = 3.0f;                ///< 最大转矩电流 [A]
    static constexpr float RATED_TORQUE_CONSTANT = 0.741f;    ///< 额定转矩常数 [mN·m/A]
    static constexpr float ORIGINAL_TRANSMISSION_RATIO = 1.0f; ///< 默认减速比 (1:1)
};

/**
 * @class DJI GM6020 云台电机类
 * @brief 实现 DJI GM6020 云台电机的基本功能
 * @details 本类继承自 DjiMotorBase，实现了 DJI GM6020 云台电机的基本功能
 * - **构造函数**：初始化 CAN 通信和电机状态
 * - **CAN 数据更新回调**：由 CAN 接收中断调用，不应在其他上下文手动调用
 * - **打印电机调试数据**：输出在线状态、角度、角速度、温度、原始电流值
 * - **设置电机输出电流**：自动钳位到 [-MAX_RAW_CURRENT, MAX_RAW_CURRENT]
 * - **设置速度反馈的低通滤波系数**：设置速度反馈的低通滤波系数
 */
class Motor6020 : public DjiMotorBase {
  public:
    /**
     * @brief GM6020 构造函数
     * @param can    硬件 CAN 对象
     * @param rx_id  RX ID = 0x204 + 电机 ID
     * @param tx_id  电机接收报文标识符，0x00 表示自动解析
     */
    Motor6020(bsp::CAN* can, uint16_t rx_id, uint16_t tx_id = 0x00);

    /**
     * @brief CAN 数据更新回调
     * @note 由 CAN 接收中断调用，不应在其他上下文手动调用
     * @param data 8 字节 CAN 数据帧 [Theta_H, Theta_L, Omega_H, Omega_L, Curr_H, Curr_L, Temp, _]
     */
    void UpdateData(const uint8_t data[]) override final;

    /**
     * @brief 打印电机调试数据
     * @note 输出在线状态、角度、角速度、温度、原始电流值
     */
    void PrintData() const override final;

    /**
     * @brief 设置电机输出电流
     * @note 自动钳位到 [-MAX_OUTPUT_CURRENT, MAX_OUTPUT_CURRENT]
     * @param val 原始电流值 [raw], raw_current ∈ [-16384, 16384] 对应转矩电流 ∈ [-3A, 3A]
     */
    void SetOutput(int16_t val) override final;

    /**
     * @brief 设置速度反馈的低通滤波系数
     * @param ratio 滤波系数 [0, 1]，越小滤波越强，默认 0.1
     */
    void SetSpeedFilter(float ratio);

  private:
    /**
     * @brief CAN 接收回调，转发至 Motor6020::UpdateData
     * @param ctx  指向 Motor6020 实例的指针
     * @param data 原始 CAN 数据
     */
    static void RxThunk(void* ctx, const uint8_t data[]);

    /**
     * @brief 由 RX_ID 自动解析 TX_ID
     * @note 在 DJI RoboMaster Assistant 中配置为电流固件控制
     * @param rx_id  RX ID = 0x204 + 电机 ID
     * @return tx_id  电流固件中 0x205-0x208 → 0x1fe, 0x209 - 0x20B→0x2fe，非零直接使用
     */
    static constexpr uint16_t ResolveTxId(uint16_t rx_id) {
        constexpr uint16_t GROUP2_RX_START = 0x209;
        constexpr uint16_t TX1_ID = 0x1fe;
        constexpr uint16_t TX2_ID = 0x2fe;
        return (rx_id >= GROUP2_RX_START) ? TX2_ID : TX1_ID;
    }

    float input_speed_filter_ = 0.1;                ///< 速度反馈低通滤波系数 [0, 1]
};

}  // namespace driver
