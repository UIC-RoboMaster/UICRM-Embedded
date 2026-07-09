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
 * @brief 达妙电机传统模式控制方式
 * @note 枚举值对应 CAN 控制帧 ID 偏移量：tx_id = motor_can_id + mode
 */
enum class DmControlMode : uint16_t {
    MIT = 0x000,      ///< MIT 模式，8 字节位域打包
    POS_VEL = 0x100,  ///< 位置速度模式，float 位置 + float 速度
    VEL = 0x200,      ///< 速度模式，float 速度，DLC=4
    EMIT = 0x300,     ///< 协议保留，驱动未实现
};

/**
 * @brief 达妙电机传统模式反馈状态（反馈帧 Byte0 高 4 位）
 */
enum class DmControlStatus : uint8_t {
    DISABLE = 0x0,          ///< 未使能
    ENABLE = 0x1,           ///< 正常运行
    OVERVOLTAGE = 0x8,      ///< 过压
    UNDERVOLTAGE = 0x9,     ///< 欠压
    OVERCURRENT = 0xA,      ///< 过流
    MOS_OVERTEMP = 0xB,     ///< MOS 过温
    ROTOR_OVERTEMP = 0xC,   ///< 线圈过温
    LOSE_CONNECTION = 0xD,  ///< 失联
    MOS_OVERLOAD = 0xE,     ///< MOS 过载
};

/**
 * @brief 达妙电机传统模式反馈（CAN 8 字节解码后的 plain 字段）
 *
 * 布局与 RoboWalker Struct_Motor_DM_CAN_Rx_Data_Normal 一致；
 * 由 DmRxFrame::Load 从原始字节显式解析，非 bitfield memcpy。
 */
struct DmRxFrame {
    uint8_t motor_id;              ///< 电机 CAN ID 低 4 位
    DmControlStatus status;          ///< 控制状态
    uint16_t raw_theta;              ///< 16-bit 编码器 raw（大端已解码）
    uint16_t raw_omega;                ///< 12-bit 速度 raw（已拼好）
    uint16_t raw_torque;               ///< 12-bit 力矩 raw（已拼好）
    uint8_t raw_mos_temp;              ///< MOS 温度 [deg C]
    uint8_t raw_rotor_temp;            ///< 线圈温度 [deg C]

    /**
     * @brief 从 CAN 接收缓冲区载入报文
     * @param data 8 字节 CAN 载荷
     */
    void Load(const uint8_t data[8]);
};

/**
 * @brief 达妙电机 TX 控制设定（三种模式共用）
 *
 * - MIT：p_des / v_des / kp / kd / t_ff 全部参与打包
 * - POS_VEL：仅 p_des / v_des
 * - VEL：仅 v_des
 */
struct DmTxFrame {
    float p_des = 0;   ///< 期望位置
    float v_des = 0;   ///< 期望速度
    float kp = 0;      ///< 
    float kd = 0;      ///< 
    float t_ff = 0;    ///< 前馈力矩 [N·m]

    /** @brief MIT 模式：更新全部字段 */
    void SetMit(float p, float v, float kp_val, float kd_val, float torque) {
        p_des = p;
        v_des = v;
        kp = kp_val;
        kd = kd_val;
        t_ff = torque;
    }

    /** @brief POS_VEL 模式：仅更新位置与速度 */
    void SetPosVel(float p, float v) {
        p_des = p;
        v_des = v;
    }

    /** @brief VEL 模式：仅更新速度 */
    void SetVel(float v) { 
        v_des = v; 
    }

    /**
     * @brief 按控制模式打包 CAN 载荷
     * @return DLC（MIT/POS_VEL=8，VEL=4）
     */
    uint8_t Pack(uint8_t data[8], DmControlMode mode, float angle_max, float omega_max, float torque_max,
                 float kp_max, float kd_max) const;
};

/**
 * @brief 达妙 (DM) 电机运行时状态
 *
 * 解析后的物理量、角度追踪、控制设定与同步标志；原始反馈见 rx。
 */
struct DmMotorState {
    DmRxFrame rx;  // 最近一次反馈（raw 整数域）

    // ── 反馈物理量 ──
    float theta = 0;   // 电机位置 [rad]
    float omega = 0;   // 电机速度 [rad/s]
    float torque = 0;  // 电机力矩 [N·m]

    // ── 输出轴 ──
    float output_shaft_theta = 0;  // 输出轴角度 [rad]
    float output_shaft_omega = 0;  // 输出轴角速度 [rad/s]

    // ── 编码器角度追踪 ──
    float power_on_angle = -1;         // 上电编码器角 [rad]（-1 未初始化）
    float encoder_relative_angle = 0;  // 编码器圈内角 [rad]，[0, 2π]
    float encoder_cumulated_turns = 0; // 编码器累计圈数 [turns]
    float encoder_cumulated_angle = 0; // 编码器累计角 [rad] = turns × 2π + encoder_relative

    // ── 输出轴角度追踪 ──
    float output_relative_angle = 0;   // 输出轴圈内角 [rad]，[0, 2π]
    float output_cumulated_turns = 0;  // 输出轴累计圈数 [turns]
    float output_cumulated_angle = 0;  // 输出轴多圈累计角 [rad] = turns × 2π + output_relative

    // ── 配置 ──
    bool enable = true;          // 软件使能
    bool absolute_mode = false;  // 绝对模式：内部仍累计圈数，output_shaft_theta 限制在 [0, 2π]

    volatile bool feedback_pending = false;  // ISR 置位，CalcOutput 消费

    // ── 控制 ──
    DmControlMode mode = DmControlMode::MIT;
    DmTxFrame tx;
};

/**
 * @brief DM 电机量程配置
 * @note 由 Dm 上位机设定 
 */
struct DmMotorConfig {
    float angle_max;            ///< 最大位置 [rad]，与上位机 PMAX 一致
    float omega_max;            ///< 最大速度 [rad/s]，与上位机 VMAX 一致
    float torque_max;           ///< 最大扭矩 [N·m]，与上位机 TMAX 一致
    float kp_max;               ///< MIT Kp 上限
    float kd_max;               ///< MIT Kd 上限
    float transmission_ratio;   ///< 减速比
};

/**
 * @brief 达妙 (DM) 电机传统模式通用基类
 *
 * 在 CanMotorBase 的基础上提供 DM 品牌电机共用的能力：
 * - 控制模式与 tx_id 管理（MIT / POS_VEL / VEL）
 * - 管理帧：使能、禁用、归零、清错（发往 motor_can_id）
 * - 后台线程周期性调用 CalcOutput() 完成反馈解析与控制帧发送
 */
class DmMotorBase : public CanMotorBase {
  public:
    /** @brief 获得电机转子角度 [rad] */
    float GetTheta() const override;

    /** @brief 获得电机转子角速度 [rad/s] */
    float GetOmega() const override;

    /** @brief 获得输出轴累计角度 [rad] */
    float GetOutputShaftTheta() const override;

    /** @brief 获得输出轴角速度 [rad/s] */
    float GetOutputShaftOmega() const override;

    /**
     * @brief 使能电机（DM 管理帧 0xFC，发往 motor_can_id）
     */
    void Enable() override;

    /**
     * @brief 禁用电机（DM 管理帧 0xFD，发往 motor_can_id）
     */
    void Disable() override;

    /** @brief 查询电机是否使能 */
    bool IsEnable() const override;

    /**
     * @brief 满足 CanMotorBase 虚接口；DM 无 DJI 式电流 raw 输出
     * @return 恒为 0（功率限制等场景请用 DJI 电机接口）
     */
    int16_t GetOutput() override;

    /**
     * @brief 不支持；DM 请使用 SetTarget() 或 SetOutput(float)
     * @note  仅为满足 CanMotorBase 纯虚接口，调用将触发断言
     */
    void SetOutput(int16_t val) override;

    /**
     * @brief 设置电机零点（DM 管理帧 0xFE，发往 motor_can_id）
     */
    virtual void SetZeroPos();

    /**
     * @brief 清除电机错误状态（DM 管理帧 0xFB，发往 motor_can_id）
     * @note  CalcOutput 在过压/过流/过温等错误态时自动调用
     */
    virtual void ClearError();

    /**
     * @brief 按当前控制模式打包并发送控制帧（发往 tx_id）
     *
     * MIT：16-bit 位置 + 12-bit 速度/力矩/Kp/Kd 位域交织，发送前 clip 并做 0x7ff 零点映射
     * POS_VEL：float 位置 + float 速度，DLC=8
     * VEL：float 速度，DLC=4
     */
    virtual void TransmitOutput();

    /**
     * @brief 后台线程周期任务：解析反馈并按 control_status 发送控制/管理帧
     *
     * 反馈 pending 时调用 FinishFeedbackUpdate（含 Heartbeat）。
     * 软件 enable 为 false 时不发送。
     * 否则按电机反馈 control_status 分支：
     * - ENABLE → TransmitOutput()
     * - DISABLE → Enable() 重试使能
     * - 错误态（过压/过流/过温/失联等）→ ClearError() 后 Enable()
     *
     * @note IsOnline() 由 Heartbeat 维护；control_status 为 DM 协议层状态，两者并存。
     */
    void CalcOutput();

    /**
     * @brief 设置 DM 电机后台线程的输出频率
     * @note 必须在首次构造 DmMotorBase 子类之前调用
     * @param freq 频率 [Hz]，默认 1000
     */
    static void SetFrequency(uint32_t freq = 1000);

    /**
     * @brief 获取电机的扭矩，单位为 [Nm]
     */
    float GetTorque() const;

    /**
     * @brief 获取电机反馈状态
     */
    DmControlStatus GetControlStatus() const;

    /**
     * @brief 更新电机的反馈数据
     * @note 由 CAN 接收中断调用，不应在其他上下文手动调用
     * @param data 原始 CAN 数据
     */
    void UpdateData(const uint8_t data[]) override;

    /**
     * @brief 设置控制模式，并更新控制帧 tx_id
     * @param mode 控制模式（MIT/POS_VEL/VEL）
     */
    void SetMode(DmControlMode mode);

    /**
     * @brief 设置 VEL 模式目标速度
     * @param target   期望速度 [rad/s]
     * @param override 保留参数，与基类接口一致
     * @note  仅在 VEL 模式下有效，否则触发断言
     */
    void SetTarget(float target, bool override = true) override;

    /**
     * @brief 设置 MIT 模式目标
     * @param position 期望位置 [rad]
     * @param velocity 期望速度 [rad/s]
     * @param kp       位置增益
     * @param kd       速度增益
     * @param t_ff     前馈力矩 [N·m]
     * @note  仅在 MIT 模式下有效，否则触发断言
     */
    void SetTarget(float position, float velocity, float kp, float kd, float t_ff);

    /**
     * @brief 设置 POS_VEL 模式目标
     * @param position 期望位置 [rad]
     * @param velocity 期望速度 [rad/s]
     * @note  仅在 POS_VEL 模式下有效，否则触发断言
     */
    void SetTarget(float position, float velocity);

  protected:
    /**
     * @brief 基础构造函数
     * @param can           CAN 对象
     * @param master_id     主控接收 ID（Master ID）
     * @param motor_can_id  电机本体 CAN ID
     * @param mode          控制模式
     * @param config        型号量程配置
     */
    DmMotorBase(bsp::CAN* can, uint16_t master_id, uint16_t motor_can_id, DmControlMode mode,
                const DmMotorConfig& config);

    DmMotorState state_;

    bsp::CAN* can_ = nullptr;       ///< CAN 硬件对象
    uint16_t rx_id_ = 0;            ///< 反馈帧 Master ID
    uint16_t tx_id_ = 0;            ///< 控制帧 CAN ID = motor_can_id + 模式偏移 电机本体 CAN ID（管理帧目标

    DmMotorConfig config_ = {};  ///< 型号量程配置（由子类传入）

    /**
     * @brief 传统模式反馈解析：raw → 物理量
     * @note UpdateData 在 ISR 内调用；解析后置位 feedback_pending，由 CalcOutput 消费
     */
    void ParseFeedbackNormal();

    /**
     * @brief 完成反馈更新：角度追踪 + 心跳
     * @note CalcOutput 在 feedback_pending 时调用；UpdateData 仅解析 raw 并置位 pending
     */
    void FinishFeedbackUpdate();

    /** @brief CAN 接收回调，转发至 DmMotorBase::UpdateData */
    static void RxThunk(void* ctx, const uint8_t data[]);

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
 * @brief DM M4310 电机配置
 *
 * 传统模式量程默认值，与上位机 PMAX/VMAX/TMAX 及 J4310 默认参数一致。
 */
struct DmMotor4310Config {
    static constexpr DmMotorConfig J4310_Config{
        .angle_max = 12.5f,
        .omega_max = 45.0f,
        .torque_max = 18.0f,
        .kp_max = 500.0f,
        .kd_max = 5.0f,
        .transmission_ratio = 1.0f,
    };
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
     * @param can           CAN 对象
     * @param master_id     主控接收 ID（Master ID）
     * @param motor_can_id  电机本体 CAN ID（上位机 CAN_ID）
     * @param mode          操作模式
     */
    DMMotor4310(bsp::CAN* can, uint16_t master_id, uint16_t motor_can_id,
                DmControlMode mode = DmControlMode::MIT);

    /**
     * @brief 打印电机调试数据
     */
    void PrintData() const override final;
};

}  // namespace driver
