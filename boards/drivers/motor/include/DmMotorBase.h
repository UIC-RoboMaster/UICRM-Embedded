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

#include "MotorCanBase.h"
#include "main.h"

namespace driver {

/**
 * @brief 达妙 4310 电机的操作模式
 */
typedef enum {
    MIT = 0,
    POS_VEL = 1,
    VEL = 2,
} dm_m4310_mode_t;

/**
 * @brief 达妙 (DM) 电机特有状态结构体
 *
 * 存放 DM 品牌独有的回传数据与设定值。
 * 所有字段集中在 DmMotorState 中，调试时一次展开即可看到全面信息。
 */
struct DmMotorState {
    // ── 编码器反馈 ──
    float theta = 0;              // 编码器角度 [rad], 范围 [0, 2PI]
    float omega = 0;              // 编码器角速度 [rad/s]

    // ── 原始回传 ──
    int16_t raw_current = 0;      // 原始电流反馈
    uint8_t raw_temperature = 0;  // 原始温度

    // ── 输出轴 ──
    float output_shaft_theta = 0; // 输出轴累计角度 [rad]
    float output_shaft_omega = 0; // 输出轴角速度 [rad/s]

    // ── 角度追踪 ──
    float power_on_angle = -1;           // 上电时的编码器角度 [rad]（-1 表示未初始化）
    float relative_angle = 0;            // 编码器相对上电角度的角度 [rad]
    float cumulated_rad = 0;             // 编码器累计圈数（2*PI/ratio 为单位） //TODO: 虽然说注释写的是编码器累计圈数，但更新时仍然除了减速比。
    int32_t cumulated_rounds = 0;        // 内部编码器累计圈数（整数圈数）
    float output_cumulated_turns = 0;    // 输出轴累计圈数（2*PI 为单位）[rad]
    float output_relative_angle = 0;     // 输出轴当前圈内角度 [rad], 范围 [0, 2PI]

    // ── 配置 ──
    float transmission_ratio = 1;  // 减速比
    bool enable = true;            // 使能
    bool absolute_mode = false;    // 绝对模式：输出轴不累计圈数

    // ── CAN 连接 ──
    bsp::CAN* can = nullptr;  // CAN 硬件对象
    uint16_t rx_id = 0;       // 接收 CAN ID
    uint16_t tx_id = 0;       // 发送 CAN ID

    // ── 时间戳 ──
    uint32_t last_update_time_us = 0;  // 最近 CAN 包时间戳

    // ── DM 专属 ──
    dm_m4310_mode_t mode = MIT;        // 操作模式（MIT/POS_VEL/VEL）
    uint16_t tx_id_actual = 0;         // 根据模式计算的实际 CAN ID

    // 反馈
    float torque = 0;                  // 反馈扭矩 [Nm]
    int16_t raw_pos = 0, raw_vel = 0, raw_torque = 0;
    uint8_t raw_mos_temp = 0, raw_motor_temp = 0;

    // 设定值
    float kp_set = 0, kd_set = 0;
    float pos_set = 0, vel_set = 0, torque_set = 0;
};

/**
 * @brief 达妙 (DM) 电机的通用基类
 *
 * 在 MotorCANBase 的基础上提供 DM 品牌电机共用的能力：
 * - CAN 发送 ID 管理
 * - 使能/禁用/归零命令（0xFC/0xFD/0xFE 通用协议）
 * - 定点数与浮点数转换工具
 */
class DmMotorBase : public MotorCANBase<DmMotorBase> {
  public:
    friend class MotorCANBase<DmMotorBase>;
    /**
     * @brief 基础构造函数
     * @param can    CAN 对象
     * @param rx_id  主控接收 ID
     * @param tx_id  CAN 发送 ID（软件配置的 ID，非实际线路 ID）
     */
    DmMotorBase(bsp::CAN* can, uint16_t rx_id, uint16_t tx_id);

    /**
     * @brief 使能电机
     */
    virtual void MotorEnable();

    /**
     * @brief 失能电机
     */
    virtual void MotorDisable();

    /**
     * @brief 设置电机零点
     */
    virtual void SetZeroPos();

    /**
     * @brief 传输数据到电机
     * @note 由具体型号实现协议打包逻辑
     */
    virtual void TransmitOutput() = 0;

    /**
     * @brief 定点数转浮点数（DM 电机协议通用工具）
     * @param x     定点数
     * @param x_min 最小值
     * @param x_max 最大值
     * @param bits  位数
     * @return 浮点数
     */
    static uint16_t float_to_uint(float x, float x_min, float x_max, int bits);

    /**
     * @brief 浮点数转定点数（DM 电机协议通用工具）
     * @param x_int 定点数
     * @param x_min 最小值
     * @param x_max 最大值
     * @param bits  位数
     * @return 浮点数
     */
    static float uint_to_float(int x_int, float x_min, float x_max, int bits);

  protected:
    DmMotorState state_;  // DM 电机全部状态数据（反馈 + 控制）
};

/**
 * @brief 达妙 4310 电机的标准类
 *
 * @note DM 4310 电机使用 MIT/POS_VEL/VEL 三种模式，
 *       每种模式下 CAN ID 和输出帧格式不同。
 */
class DMMotor4310 : public DmMotorBase {
  public:
    using MotorBase::SetOutput;

    /**
     * @brief 基础构造函数
     * @param can    CAN 对象
     * @param rx_id  主控接收 ID
     * @param tx_id  软件配置的 CAN ID
     * @param mode   操作模式
     */
    DMMotor4310(bsp::CAN* can, uint16_t rx_id, uint16_t tx_id, dm_m4310_mode_t mode);

    /**
     * @brief 更新电机的反馈数据
     * @note 仅在 CAN 回调函数中使用
     * @param data 原始数据
     */
    void UpdateData(const uint8_t data[]);

    /**
     * @brief 传输数据到电机（根据模式打包）
     */
    void TransmitOutput() override;

    /**
     * @brief 获取电机的扭矩，单位为 [Nm]
     */
    float GetTorque() const;

    /**
     * @brief 打印电机数据
     */
    void PrintData() const override;

    /**
     * @brief 设置电机的输出参数（MIT 模式）
     * @param position 角度 [rad]
     * @param velocity 角速度 [rad/s]
     * @param kp       KP 值
     * @param kd       KD 值
     * @param torque   扭矩 [Nm]
     */
    void SetOutput(float position, float velocity, float kp, float kd, float torque);

    /**
     * @brief 设置电机的输出参数（POS_VEL 模式）
     * @param position 角度 [rad]
     * @param velocity 角速度 [rad/s]
     */
    void SetOutput(float position, float velocity);

    /**
     * @brief 设置电机的输出参数（VEL 模式）
     * @param velocity 角速度 [rad/s]
     */
    void SetOutput(float velocity);

    // DM m4310 量程常量
    static constexpr float P_MIN = -12.5f;
    static constexpr float P_MAX = 12.5f;
    static constexpr float V_MIN = -45.0f;
    static constexpr float V_MAX = 45.0f;
    static constexpr float T_MIN = -18.0f;
    static constexpr float T_MAX = 18.0f;
    static constexpr float KP_MIN = 0.0f;
    static constexpr float KP_MAX = 500.0f;
    static constexpr float KD_MIN = 0.0f;
    static constexpr float KD_MAX = 5.0f;
};

}  // namespace driver
