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
#include <unordered_map>

#include "CanMotorBase.h"
#include "bsp_thread.h"
#include "pid.h"
#include "utils.h"

namespace driver {

/**
 * @brief DJI 电机特有状态结构体
 *
 */
struct DjiMotorState {
    // ── 编码器反馈 ──
    float theta = 0;              // 编码器角度 [rad], 范围 [0, 2PI]
    float omega = 0;              // 编码器角速度 [rad/s]
    float current = 0;            // 电机电流 [A]
    float temperature = 0;        // 电机温度 [°C]
    float torque = 0;             // 电机扭矩 [Nm]

    // ── 电机原始回传 ──
    int16_t raw_theta = 0;        // 转子机械角度（原始编码器角度）
    int16_t raw_omega = 0;        // 转子转速（原始编码器角速度）
    int16_t raw_current = 0;      // 原始电流反馈
    uint8_t raw_temperature = 0;  // 原始温度

    // ── 输出轴 ──
    float output_shaft_theta = 0; // 输出轴累计角度 [rad]
    float output_shaft_omega = 0; // 输出轴角速度 [rad/s]

    // ── 编码器角度追踪 ──
    float power_on_angle = -1;         // 上电时的编码器角度 [rad]（-1 表示未初始化）
    float encoder_relative_angle = 0;  // 编码器当前圈内角度 [rad], 范围 [0, 2π]
    float encoder_cumulated_turns = 0; // 编码器累计圈数 [turns]
    float encoder_cumulated_angle = 0; // 编码器累计角度 [rad] = turns × 2π + encoder_relative

    // ── 输出轴角度追踪 ──
    float output_relative_angle = 0;   // 输出轴当前圈内角度 [rad], 范围 [0, 2π]
    float output_cumulated_turns = 0;  // 输出轴累计圈数 [turns]
    float output_cumulated_angle = 0;  // 输出轴多圈累计角度 [rad] = turns × 2π + output_relative

    // ── 配置 ──
    uint8_t mode = 0;              // 控制模式（OMEGA/THETA/ABSOLUTE/INVERTED）
    float transmission_ratio = 1;  // 减速比
    bool enable = true;            // 使能
    bool absolute_mode = false;    // 绝对模式：内部仍累计圈数，output_shaft_theta 限制在 [0, 2π]

    // ── 时间戳 ──
    uint32_t last_update_time_us = 0;  // 最近 CAN 包时间戳
    uint32_t motor_update_time_interval;   // CAN 回传间隔 [us]

    volatile bool feedback_pending = false;  // 是否收到新的反馈

    // ── DJI 专属控制字段 ──
    float target = 0;                      // 目标值：角度 [rad] 或 角速度 [rad/s]
    float speed_offset = 0;                // 前馈速度偏移 [rad/s]，叠加到 ω 环设定
    float torque_feedforward = 0;          // 前馈力矩 [N·m]，叠加到 ω 环电流输出
    float proximity_in = 0.05;             // 进入保持状态的临界角度差
    float proximity_out = 0.15;            // 退出保持状态的临界角度差
    bool holding = true;                   // 角度模式下是否已达目标
    
};

/**
 * @brief DJI 品牌 CAN 电机的基类
 *
 * 在 CanMotorBase 的基础上添加 DJI 电机特有的能力：
 * - 级联 PID 控制（角度环 + 速度环）
 * - CAN 分组传输（4 电机共享 1 个 TX ID）
 * - 后台线程自动输出
 * - 堵转回调
 * - 前馈偏移
 */
class DjiMotorBase : public CanMotorBase {
  public:
    enum motor_mode {
        // 未使用
        NONE = 0x00,
        // 开环电流指令：state_.target 为 raw 电流 setpoint，旁路 PID
        CURRENT = 0x01,
        // 启用速度环控制
        OMEGA = 0x02,
        // 启用角度环控制
        THETA = 0x04,
        // 反转电机方向
        INVERTED = 0x40,
        // ABSOLUTE 模式下，对外输出轴角度限制在 [0, 2π]；内部仍累计圈数，
        // 若目标在相反半圈则从另一侧绕过去
        ABSOLUTE = 0x80,
    };

    /**
     * @brief 堵转回调函数模板
     */
    typedef void (*callback_t)(void* instance);

    /**
     * @brief 基础构造函数
     * @param can    CAN 对象
     * @param rx_id  电机使用的 CAN 接收 ID，参考电机的说明书
     * @param tx_id  电机使用的 CAN 发送 ID（各子类负责各自的自动识别逻辑）
     */
    DjiMotorBase(bsp::CAN* can, uint16_t rx_id, uint16_t tx_id = 0x00);

    ~DjiMotorBase() override;

    /**
     * @brief 设置 DJI 电机后台线程的输出频率
     * @note 必须在首次构造 DJI 电机子类之前调用
     * @param freq 频率 [Hz]，默认 1000
     */
    static void SetFrequency(uint32_t freq = 1000);

    /** @brief 获得电机转子角度 [rad] */
    float GetTheta() const override;

    /** @brief 获取编码器累计角度 [rad]（多圈累计，无回绕） */
    float GetEncoderCumulatedTheta() const;

    /** @brief 获得电机转子角速度 [rad/s] */
    float GetOmega() const override;

    /** @brief 获得输出轴累计角度 [rad] */
    float GetOutputShaftTheta() const override;

    /** @brief 获取输出轴累计角度 [rad]（多圈累计，无回绕） */
    float GetOutputShaftCumulatedTheta() const;

    /** @brief 获得输出轴角速度 [rad/s] */
    float GetOutputShaftOmega() const override;

    /**
     * @brief 获取反馈力矩 [N·m]（由电流反馈 × Kt 得到）
     */
    float GetTorque() const;

    /** @brief 使能电机输出 */
    void Enable() override;

    /** @brief 禁用电机输出 */
    void Disable() override;

    /** @brief 查询电机是否使能 */
    bool IsEnable() const override;

    /**
     * @brief 获取当前输出电流指令
     * @return 输出电流值 [raw]
     */
    int16_t GetOutput() override;

    /**
     * @brief 读取上一次设置的目标值
     * @return 输出轴的目标：角度 [RAD]、累计角度 [RAD]、角速度 [RAD/S]（取决于模式）
     */
    float GetTarget() const;

    /**
     * @brief 获得原始电流反馈
     * @return 原始电流值 [raw]
     */
    int16_t GetCurr() const;

    /**
     * @brief 获得原始温度反馈
     * @return 原始温度
     */
    uint16_t GetTemp() const;

    /**
     * @brief 在角度控制模式下，是否已经达到目标角度。
     */
    bool IsHolding() const;

    /**
     * @brief 在角度控制模式下，使电机停止在当前位置。
     * @param override 应为 true
     */
    void Hold(bool override = true);

    /**
     * @brief 在 ω 环设定值上叠加前馈角速度
     * @note 用于底盘同步等速度前馈；力矩前馈请用 SetTorqueFeedforward
     */
    void SetSpeedOffset(float offset);

    /**
     * @brief 在 ω 环电流输出上叠加前馈力矩 [N·m]
     * @note 须启用 OMEGA；I_cmd = PID_ω(...) + τ_ff/Kt，C620 内环负责跟踪电流
     */
    void SetTorqueFeedforward(float torque_nm);

    /**
     * @brief 开环力矩指令 [N·m]，旁路 PID
     * @note 须 SetMode(CURRENT)；级联控制下的力矩前馈请用 SetTorqueFeedforward
     * @param override 同 SetTarget
     */
    void SetTorque(float torque_nm, bool override = true);

    /** @brief 获取减速比 */
    float GetTransmissionRatio() const;

    /**
     * @brief 设置减速比
     * @param ratio 减速比，必须大于 0
     */
    void SetTransmissionRatio(float ratio);

    /**
     * @brief
     * 设置电机的工作模式，工作模式由若干个 bool 值组成，请参考电机模式的定义，启动多个模式的情况需要使用或运算
     * @param mode 电机的工作模式
     */
    void SetMode(uint8_t mode);

    /**
     * @brief 设置绝对模式
     * @param enable true 表示对外输出限制在 [0, 2π]（内部仍累计圈数）
     */
    void SetAbsoluteMode(bool enable);

    /**
     * @brief 更新电机的反馈数据
     * @note 仅由子类实现；在 CAN 接收回调中调用，不要在其他地方调用
     * @param data[]  原始数据
     */
    void UpdateData(const uint8_t data[]) override;

    /**
     * @brief 更新电机的保持状态（DJI 模式专用逻辑）
     */
    void UpdateHoldingState();

    /**
     * @brief 设置目标
     * @param target 设置输出轴的目标：角度 [RAD]、累计角度 [RAD]、角速度 [RAD/S]（取决于模式）
     * @param override 电机为角度控制模式下，还未达到之前的目标时，是否覆盖旧的目标
     */
    void SetTarget(float target, bool override = true) override;

    /**
     * @brief 通过电机的 pid 控制器计算电机的输出
     * @note 本函数会在电机输出进程中按照所设定的频率被自动调用，正常情况下请勿手动调用
     */
    void CalcOutput();

    /**
     * @brief 设置电机的 PID
     * @param pid_init pid 的初始化参数
     * @param mode 所需要设置的 pid 的环，一般是速度环或者角度环
     */
    void ReInitPID(control::ConstrainedPID::PID_Init_t pid_init, uint8_t mode);

    /**
     * @brief 获取电机 PID 数值
     */
    control::ConstrainedPID::PID_State_t GetPIDState(uint8_t mode) const;

    /**
     * @brief 设置 ServoMotor 为 DjiMotorBase 的友元，因为它们需要使用 DjiMotorBase 的许多私有参数。
     */
    friend class ServoMotor;

    /**
     * @brief 设置堵转回调函数
     */
    void RegisterErrorCallback(callback_t callback, void* instance);

    /**
     * @brief 处理堵转回调函数
     * @param instance 关联的电机实例
     * @param type pid 的故障类型
     */
    static void ErrorCallbackWrapper(void* instance,
                                     control::ConstrainedPID::PID_ErrorHandler_t type);

    /**
     * @brief 设置在执行输出数据前的回调函数，一般用于功率限制
     */
    static void RegisterPreOutputCallback(callback_t callback, void* instance);

    /**
     * @brief 设置在执行输出数据后的回调函数，一般用于垃圾清理等
     */
    static void RegisterPostOutputCallback(callback_t callback, void* instance);

  protected:
    DjiMotorState state_;  // 电机全部状态数据（反馈 + 控制）

    bsp::CAN* can_ = nullptr;  // CAN 硬件对象
    uint16_t rx_id_ = 0;       // 电机反馈报文标识符
    uint16_t tx_id_ = 0;       // 电机控制报文标识符

    int16_t output_ = 0;   // 当前输出电流指令 [raw]

    /// 转矩常数 [N·m/A]，由各子类构造函数根据电机规格设置
    float torque_constant_ = 0;

    /// 电流物理量程 [A]、原始码值量程 [raw]，用于力矩 - 电流 raw 换算
    float max_current_amp_ = 0;
    int16_t max_raw_current_ = 0;

    /**
     * @brief 单圈绝对值编码器的角度追踪处理
     * @note 子类在 UpdateData 中解析完协议后，经 FeedbackUpdate 调用此方法
     * @note absolute_mode 下内部仍累计圈数，output_shaft_theta 对外限制在 [0, 2π]
     * @warning 这是使用单圈绝对值编码器的电机的角度处理，不通用于多圈编码器
     */
    void AngleTracking();

    /**
     * @brief 完成反馈更新：角度追踪 + 心跳 + 保持状态
     * @note CalcOutput 开头在 feedback_pending 时调用；ISR 内 UpdateData 仅置位 pending
     */
    void FeedbackUpdate();

    /// 编码器 raw theta [0, 2π] 回绕检测（2π↔0）
    FloatEdgeDetector* inner_wrap_detector_;
    /// 输出轴圈内角 [0, 2π] 回绕检测
    FloatEdgeDetector* outer_wrap_detector_;

  private:
    control::ConstrainedPID omega_pid_;
    control::ConstrainedPID theta_pid_;

    callback_t error_callback_ = [](void* instance) { UNUSED(instance); };
    void* error_callback_instance_ = nullptr;

    /**
     * @brief DJI CAN 电机分组结构体
     *
     * DJI 协议同一 TX ID 最多承载 4 个电机（8 字节 = 4 × int16_t），
     * 相同 (TX ID, CAN 总线) 的电机归为一组，共享一帧 CAN 报文。
     */
    struct MotorGroup {
        bool occupied = false;          // 槽位是否已占用（memset(0) 后天然为 false）
        uint16_t tx_id = 0;            // 组的 CAN 发送 ID
        bsp::CAN* can = nullptr;       // 组的 CAN 总线
        DjiMotorBase* motors[4] = {};  // 组内电机指针（最多 4 个）
        uint8_t count = 0;             // 组内实际电机数
    };

    /**
     * @brief 发送 CAN 消息以设置电机输出（一帧 = 一个 group 的全部电机）
     * @param group  电机分组
     */
    static void TransmitOutput(const MotorGroup& group);

    static bool is_init_;

    static bsp::Thread* can_motor_thread_;
    static constexpr osThreadAttr_t can_motor_thread_attr_ = {
        .name = "MotorUpdateTask",
        .attr_bits = osThreadDetached,
        .cb_mem = nullptr,
        .cb_size = 0,
        .stack_mem = nullptr,
        .stack_size = 256 * 4,
        .priority = (osPriority_t)osPriorityHigh,
        .tz_module = 0,
        .reserved = 0};

    static void CanMotorThread(void* args);

    /**
     * @brief DJI CAN 电机分组注册表
     *
     * 一个 group = (TX ID, CAN 总线) 二元组。同组电机共享一帧 CAN 报文（最多 4 个）。
     * 不同 TX ID 或不同 CAN 总线即创建新 group。
     *
     * Group 从 0 开始连续存放、从不删除，group_count_ 即已占用槽位数。
     * DJI 协议仅 3 个 TX ID（0x200 / 0x1FF / 0x2FF），即使 2 条 CAN 全用也仅 6 组，[10] 为预留值。
     */
    static MotorGroup groups_[10];
    static uint8_t group_count_;
    static uint32_t delay_time;

    static callback_t pre_output_callback_;
    static void* pre_output_callback_instance_;
    static callback_t post_output_callback_;
    static void* post_output_callback_instance_;
};


// ===== Motor3508 =====

/**
 * @brief DJI M3508/P19 减速电机默认配置
 * @details 本结构体包含了电机编码器、电调电流映射、减速比以及转矩常数等核心物理参数
 * - **电调映射**：-16384 ~ 16384 对应 -20A ~ 20A
 * - **编码器**：0 ~ 8191 对应 0 ~ 360°
 * - **减速比**：3591:187 ≈ 19.2:1
 */
 struct Motor3508Config {
  static constexpr int16_t MAX_RAW_THETA = 8191;                     ///< 转子机械角最大值 [raw] 0->8191 对应 0~360°
  static constexpr int16_t MAX_RAW_CURRENT = 16384;                  ///< 转矩电流反馈最大值 [raw] -16384->16384 对应 -20A~20A
  static constexpr float MAX_CURRENT = 20.0f;                        ///< 最大转矩电流 [A]
  static constexpr float RATED_TORQUE_CONSTANT = 0.3f;               ///< 额定转矩常数 [mN·m/A]
  static constexpr float ORIGINAL_TRANSMISSION_RATIO = 3591.0f / 187.0f; ///< 默认减速比 (19.2:1)
};

/**
* @class DJI M3508/P19 减速电机类
* @brief 实现 DJI M3508/P19 减速电机的基本功能
* @details 本类继承自 DjiMotorBase，实现了 DJI M3508/P19 减速电机的基本功能
* - **构造函数**：初始化 CAN 通信和电机状态
* - **CAN 数据更新回调**：由 CAN 接收中断调用，不应在其他上下文手动调用
* - **打印电机调试数据**：输出在线状态、角度、角速度、原始电流值
* - **设置电机输出电流**：自动钳位到 [-MAX_RAW_CURRENT, MAX_RAW_CURRENT]
*/
class Motor3508 : public DjiMotorBase {
public:
  /**
   * @brief M3508 构造函数
   * @param can    硬件 CAN 对象
   * @param rx_id  RX ID = 0x200 + 电机 ID
   * @param tx_id  电调接收报文标识符，0x00 表示自动解析
   */
  Motor3508(bsp::CAN* can, uint16_t rx_id, uint16_t tx_id = 0x00);

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
   * @note 自动钳位到 [-MAX_RAW_CURRENT, MAX_RAW_CURRENT]
   * @param val 原始电流值 [raw], raw_current ∈ [-16384, 16384] 对应转矩电流 ∈ [-20A, 20A]
   */
  void SetOutput(int16_t val) override final;

private:
  /**
   * @brief CAN 接收回调，转发至 Motor3508::UpdateData
   * @param ctx  指向 Motor3508 实例的指针
   * @param data 原始 CAN 数据
   */
  static void RxThunk(void* ctx, const uint8_t data[]);

  /**
   * @brief 由 RX_ID 自动解析 TX_ID
   * @note M3508 + C620 电调标准 CAN 协议标识符
   * @param rx_id  RX ID = 0x200 + 电调 ID
   * @return tx_id  由 rx_id 决定: 0x201 - 0x204 → 0x200, 0x205 - 0x208 → 0x1ff
   */
  static constexpr uint16_t ResolveTxId(uint16_t rx_id) {
      return (rx_id >= 0x205) ? 0x1ff : 0x200;
  }
};


// ===== Motor6020 =====

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


// ===== Motor2006 =====

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

};


// ===== DmMotor1—to-4 =====

/**
 * @brief Dm 一拖四固件 电机默认配置
 * @details 本结构体包含了电机编码器、电调电流映射、减速比以及转矩常数等核心物理参数
 * - **电调映射**：-16384 ~ 16384 对应 -20.5A ~ 20.5A
 * - **编码器**：0 ~ 8191 对应 0 ~ 360°
 * - **减速比**：3591:187 ≈ 19.2:1
 */
 struct Dm1to4Config {
  static constexpr int16_t MAX_RAW_THETA = 8191;                     ///< 转子机械角最大值 [raw] 0->8191 对应 0~360°
  static constexpr int16_t MAX_RAW_CURRENT = 16384;                  ///< 转矩电流反馈最大值 [raw] -16384->16384 对应 -20.5A~20.5A
  static constexpr float MAX_CURRENT = 20.5f;                        ///< 最大转矩电流 [A]
  static constexpr float RATED_TORQUE_CONSTANT = 0;                  ///< 未知 额定转矩常数 [mN·m/A] 
  static constexpr float ORIGINAL_TRANSMISSION_RATIO = 1;            ///< 默认减速比 
};

/**
* @class 
* @brief Dm 通过上位机刷一拖四固件，此固件完美兼容 dji c620 电调协议
* @details 本类继承自 DjiMotorBase，实现了 Dm 一拖四固件 电机的基本功能
* - **构造函数**：初始化 CAN 通信和电机状态
* - **CAN 数据更新回调**：由 CAN 接收中断调用，不应在其他上下文手动调用
* - **打印电机调试数据**：输出在线状态、角度、角速度、原始电流值
* - **设置电机输出电流**：自动钳位到 [-MAX_RAW_CURRENT, MAX_RAW_CURRENT]
*/
class Dm1to4 : public DjiMotorBase {
public:
  /**
   * @brief Dm1to4 构造函数
   * @param can    硬件 CAN 对象
   * @param rx_id  RX ID = 0x200 + 电机 ID
   * @param tx_id  电调接收报文标识符，0x00 表示自动解析
   */
  Dm1to4(bsp::CAN* can, uint16_t rx_id, uint16_t tx_id = 0x00);

  /**
   * @brief CAN 数据更新回调
   * @note 由 CAN 接收中断调用，不应在其他上下文手动调用
   * @param data 8 字节 CAN 数据帧 [Theta_H, Theta_L, Omega_H, Omega_L, Curr_H, Curr_L, Temp, Error]
   */
  void UpdateData(const uint8_t data[]) override final;

  /**
   * @brief 打印电机调试数据
   * @note 输出在线状态、角度、角速度、温度、原始电流值
   */
  void PrintData() const override final;

  /**
   * @brief 设置电机输出电流
   * @note 自动钳位到 [-MAX_RAW_CURRENT, MAX_RAW_CURRENT]
   * @param val 原始电流值 [raw], raw_current ∈ [-16384, 16384] 对应转矩电流 ∈ [-20.5A, 20.5A]
   */
  void SetOutput(int16_t val) override final;

private:
  /**
   * @brief CAN 接收回调，转发至 Dm1to4::UpdateData
   * @param ctx  指向 Dm1to4 实例的指针
   * @param data 原始 CAN 数据
   */
  static void RxThunk(void* ctx, const uint8_t data[]);

  /**
   * @brief 由 RX_ID 自动解析 TX_ID
   * @note M3508 + C620 电调标准 CAN 协议标识符
   * @param rx_id  RX ID = 0x200 + 电调 ID
   * @return tx_id  由 rx_id 决定: 0x201 - 0x204 → 0x200, 0x205 - 0x208 → 0x1ff
   */
  static constexpr uint16_t ResolveTxId(uint16_t rx_id) {
      return (rx_id >= 0x305) ? 0x3fe : 0x4fe;
  }
};

}  // namespace driver
