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
#include "bsp_thread.h"
#include "main.h"

namespace driver {

/**
 * @brief 达妙电机的控制模式
 * @note 该枚举值对应 DM 电机的 CAN 控制帧 ID 偏移量，详见 m4310 文档。
 */
typedef enum {
    MIT = 0x000,
    POS_VEL = 0x100,
    VEL = 0x200,
} dm_mode_t;

/**
 * @brief 达妙 (DM) 电机特有状态结构体
 *
 * 存放 DM 品牌独有的回传数据与设定值。
 * 所有字段集中在 DmMotorState 中，调试时一次展开即可看到全面信息。
 */
struct DmMotorState {
    // ── 反馈物理量──
    float theta = 0;                // POS 电机位置 [rad]
    float omega = 0;                // VEL 电机速度 [rad/s]
    float torque = 0;               // T 电机扭矩 [Nm]


    // ── 原始回传 ──
    uint8_t id = 0;                 // ID 控制器 ID（CAN ID 低 4 位）
    uint8_t error = 0;                // ERR 状态码
    uint16_t raw_position = 0;      // 原始电机位置 POS [rad]
    int16_t raw_velocity = 0;       // 原始电机速度信息 VEL [rad/s]
    uint16_t raw_torque = 0;        // 原始电机力矩信息 T [Nm]
    uint8_t mos_temperature = 0;    // T_MOS 驱动 MOS 平均温度 [deg C]
    uint8_t rotor_temperature = 0;  // T_Rotor 电机线圈平均温度 [deg C]
    

    // ── 输出轴 ──
    float output_shaft_theta = 0; // 输出轴累计角度 [rad]
    float output_shaft_omega = 0; // 输出轴角速度 [rad/s]

    // ── 角度追踪 ──
    float power_on_angle = -1;           // 上电时的编码器角度 [rad]（-1 表示未初始化）
    float relative_angle = 0;            // 编码器相对上电角度的角度 [rad]
    float output_cumulated_angle = 0;    // 输出轴累计角度 [rad]（由回绕事件 ±2PI 累加）
    float output_relative_angle = 0;     // 输出轴当前圈内角度 [rad], 范围 [0, 2PI]

    // ── 配置 ──
    float transmission_ratio = 1;  // 减速比
    bool enable = true;            // 使能
    bool absolute_mode = false;    // 绝对模式：输出轴不累计圈数

    // ── CAN 连接 ──
    bsp::CAN* can = nullptr;  // CAN 硬件对象
    uint16_t rx_id = 0;       // 反馈帧 电机内设定的 Master ID
    uint16_t tx_id = 0;       // 控制帧 电机内设定的 CAN ID

    // ── 时间戳 ──
    uint32_t last_update_time_us = 0;  // 最近 CAN 包时间戳

    // ── DM 控制配置 ──
    dm_mode_t mode;        // 操作模式（MIT/POS_VEL/VEL）

    // ── MIT 控制帧设定值 ──
    float position_setpoint = 0;        // p_des 期望位置 [rad]
    float velocity_setpoint = 0;        // v_des 期望速度 [rad/s]
    float kp_setpoint = 0;              // Kp 位置增益
    float kd_setpoint = 0;              // Kd 速度增益
    float torque_feedforward = 0;       // t_ff 前馈扭矩 [Nm]
};

/**
 * @brief 达妙 (DM) 电机的通用基类
 *
 * 在 MotorCANBase 的基础上提供 DM 品牌电机共用的能力：
 * - CAN 发送 ID 管理
 * - 使能/禁用/归零命令（0xFC/0xFD/0xFE 通用协议）
 */
class DmMotorBase : public MotorCANBase<DmMotorBase> {
  public:
    friend class MotorCANBase<DmMotorBase>;
    using MotorBase::SetOutput;
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
     * @brief 计算并传输输出（由后台线程定期调用）
     */
    void CalcOutput();

    /**
     * @brief 设置 DM 电机后台线程的输出频率
     * @note 必须在首次构造 DmMotorBase 子类之前调用
     * @param freq 频率 [Hz]，默认 1000
     */
    static void SetOutputFrequency(uint32_t freq = 1000);

    /**
     * @brief 获取电机的扭矩，单位为 [Nm]
     */
    float GetTorque() const;

    /**
     * @brief 设置电机的输出参数（MIT 模式）
     * @note  手动模式使用；自动模式请用 SetTarget
     */
    void SetOutput(float position, float velocity, float kp, float kd, float torque);

    /**
     * @brief 设置电机的输出参数（POS_VEL 模式）
     */
    void SetOutput(float position, float velocity);

    /**
     * @brief 设置电机的输出参数（VEL 模式）
     */
    void SetOutput(float velocity);

    /**
     * @brief 设置电机目标（VEL 模式单参数快捷方式）
     * @note  重写基类虚函数，仅在 VEL 模式下有效，否则触发断言
     */
    void SetTarget(float target, bool override = true) override;

    /**
     * @brief 设置电机目标（MIT 模式）
     * @note  仅在 MIT 模式下有效，否则触发断言
     */
    void SetTarget(float position, float velocity, float kp, float kd, float t_ff);

    /**
     * @brief 设置电机目标（POS_VEL 模式）
     * @note  仅在 POS_VEL 模式下有效，否则触发断言
     */
    void SetTarget(float position, float velocity);

  protected:
    DmMotorState state_;  // DM 电机全部状态数据（反馈 + 控制）

  private:
    // ── 后台线程基础设施 ──

    /** @brief 后台线程入口：定时遍历所有 DM 实例并调用 CalcOutput() */
    static void DmMotorThread(void* args);

    /** @brief 全局 DM 电机实例注册表（最多 16 台） */
    static DmMotorBase* instances_[16];
    static uint8_t instance_count_;
    static bool dm_thread_started_;
    static bsp::Thread* dm_thread_;
    static uint32_t dm_output_period_us_;
};

/**
 * @brief 达妙 4310 电机的标准类
 *
 * @note DM 4310 电机使用 MIT/POS_VEL/VEL 三种模式，
 *       每种模式下 CAN ID 和输出帧格式不同。
 */
class DMMotor4310 : public DmMotorBase {
  public:
    /**
     * @brief 基础构造函数
     * @param can    CAN 对象
     * @param rx_id  主控接收 ID
     * @param tx_id  软件配置的 CAN ID
     * @param mode   操作模式
     */
    DMMotor4310(bsp::CAN* can, uint16_t rx_id, uint16_t tx_id, dm_mode_t mode);

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
     * @brief 打印电机数据
     */
    void PrintData() const override;

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
    static constexpr int POS_BITS = 16;
    static constexpr uint16_t POS_MAX_RAW = (1u << POS_BITS) - 1u;
    static constexpr int MIT_PARAM_BITS = 12;
    static constexpr uint16_t MIT_PARAM_MAX_RAW = (1u << MIT_PARAM_BITS) - 1u;
};

}  // namespace driver
