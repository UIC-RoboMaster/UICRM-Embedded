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

#include "MotorCanBase.h"
#include "bsp_thread.h"
#include "pid.h"
#include "utils.h"

#define M3508P19_MAX_OUTPUT 12000.0f

namespace driver {

/**
 * @brief DJI 电机特有状态结构体
 *
 * 扩展 MotorState（基类 state_），存放 DJI 品牌 PID 控制相关的专属字段。
 */
struct DjiMotorState {
    // ── 编码器反馈 ──
    float theta = 0;              // 编码器角度 [rad], 范围 [0, 2PI]
    float omega = 0;              // 编码器角速度 [rad/s]

    // ── 电机原始回传 ──
    int16_t raw_theta = 0;        // 转子机械角度（原始编码器角度）
    int16_t raw_omega = 0;        // 转子转速（原始编码器角速度）
    int16_t raw_current = 0;      // 原始电流反馈
    uint8_t raw_temperature = 0;  // 原始温度

    // ── 输出轴 ──
    float output_shaft_theta = 0; // 输出轴累计角度 [rad]
    float output_shaft_omega = 0; // 输出轴角速度 [rad/s]

    // ── 编码器角度追踪 ──
    float power_on_angle = -1;           // 上电时的编码器角度 [rad]（-1 表示未初始化）
    float relative_angle = 0;            // 编码器相对上电角度的角度 [rad]

    // ── 输出轴角度追踪 ──
    float output_cumulated_angle = 0;    // 输出轴累计角度 [rad]（由回绕事件 ±2PI 累加）
    float output_relative_angle = 0;     // 输出轴当前圈内角度 [rad], 范围 [0, 2PI]

    // ── 配置 ──
    uint8_t mode = 0;              // 控制模式（OMEGA/THETA/ABSOLUTE/INVERTED）
    float transmission_ratio = 1;  // 减速比
    bool enable = true;            // 使能
    bool absolute_mode = false;    // 绝对模式：输出轴不累计圈数

    // ── CAN 连接 ──
    bsp::CAN* can = nullptr;  // CAN 硬件对象
    uint16_t rx_id = 0;       // 电机反馈报文标识符
    uint16_t tx_id = 0;       // 电机接收报文标识符

    // ── 时间戳 ──
    uint32_t last_update_time_us = 0;  // 最近 CAN 包时间戳
    uint32_t motor_update_time_interval;   // CAN 回传间隔 [us]

    // ── DJI 专属控制字段 ──
    float target = 0;                      // 目标值：角度 [rad] 或 角速度 [rad/s]
    float speed_offset = 0;                // 前馈速度偏移
    float proximity_in = 0.05;             // 进入保持状态的临界角度差
    float proximity_out = 0.15;            // 退出保持状态的临界角度差
    bool holding = true;                   // 角度模式下是否已达目标
    
};

/**
 * @brief DJI 品牌 CAN 电机的基类
 *
 * 在 MotorCANBase 的基础上添加 DJI 电机特有的能力：
 * - 级联 PID 控制（角度环 + 速度环）
 * - CAN 分组传输（4 电机共享 1 个 TX ID）
 * - 后台线程自动输出
 * - 堵转回调
 * - 前馈偏移
 */
class DjiMotorBase : public MotorCANBase<DjiMotorBase> {
  public:
    friend class MotorCANBase<DjiMotorBase>;

    enum motor_mode {
        // 未使用
        NONE = 0x00,
        // 未使用
        CURRENT = 0x01,
        // 启用速度环控制
        OMEGA = 0x02,
        // 启用角度环控制
        THETA = 0x04,
        // 反转电机方向
        INVERTED = 0x40,
        // ABSOLUTE 模式下，认为输出轴只有一圈。电机输出轴角度不会累计，被限制在 [0,
        // 2PI] 之间，如果目标在相反的半圈，则从另一侧绕过去
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

    static void SetFrequency(uint32_t freq = 1000);

    /**
     * @brief 更新电机的反馈数据
     * @note 仅在 CAN 回调函数中使用，不要在其他地方调用
     * @param data[]  原始数据
     */
    void UpdateData(const uint8_t data[]) override;

    /**
     * @brief 更新电机的保持状态（DJI 模式专用逻辑）
     */
    void UpdateHoldingState() override;

    /**
     * @brief 通过电机的 pid 控制器计算电机的输出
     * @note 本函数会在电机输出进程中按照所设定的频率被自动调用，正常情况下请勿手动调用
     */
    virtual void CalcOutput();

    /**
     * @brief 设置目标
     * @param target 设置输出轴的目标：角度 [RAD]、累计角度 [RAD]、角速度 [RAD/S]（取决于模式）
     * @param override 电机为角度控制模式下，还未达到之前的目标时，是否覆盖旧的目标
     */
    virtual void SetTarget(float target, bool override = true);

    /**
     * @brief 读取上一次设置的目标值
     * @return 输出轴的目标：角度 [RAD]、累计角度 [RAD]、角速度 [RAD/S]（取决于模式）
     */
    virtual float GetTarget() const;

    /**
     * @brief 设置电机的 PID
     * @param pid_init pid 的初始化参数
     * @param mode 所需要设置的 pid 的环，一般是速度环或者角度环
     */
    virtual void ReInitPID(control::ConstrainedPID::PID_Init_t pid_init, uint8_t mode);

    /**
     * @brief 获取电机 PID 数值
     */
    virtual control::ConstrainedPID::PID_State_t GetPIDState(uint8_t mode) const;

    /**
     * @brief
     * 设置电机的工作模式，工作模式由若干个 bool 值组成，请参考电机模式的定义，启动多个模式的情况需要使用或运算
     * @param mode 电机的工作模式
     */
    void SetMode(uint8_t mode);

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
     * @brief 在电机目标速度（角度环 PID 的输出）上加上一个偏移量
     * @note 用于实现前馈
     */
    void SetSpeedOffset(float offset);

    /**
     * @brief 设置电机扭矩（力矩控制模式）
     * @note 需先通过 SetMode(CURRENT) 切换到力矩控制模式
     * @param torque_nm 目标扭矩 [N·m]，内部换算：raw = torque_nm * 1000 / (torque_constant_ * RAW_CURRENT_TO_AMP)
     * @param override 是否覆盖之前的目标
     */
    void SetTorque(float torque_nm, bool override = true);

    /**
     * @brief 获取当前电机扭矩
     * @return 当前扭矩 [N·m]，换算公式：raw_current * RAW_CURRENT_TO_AMP * torque_constant_ / 1000
     */
    float GetTorque() const;

  protected:
    DjiMotorState state_;  // 电机全部状态数据（反馈 + 控制）

    /// DJI CAN 协议：raw_current ∈ [-16384, 16384] 对应转矩电流 ∈ [-3A, 3A]
    static constexpr float RAW_CURRENT_TO_AMP = 3.0f / 16384.0f;

    /// 转矩常数 [mN·m/A]，由各子类构造函数根据电机规格设置
    float torque_constant_ = 0;

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

    static const int16_t MAX_OUT = 32767;

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





/**
 * @brief 伺服电机旋转模式，用于 DJI 的 CAN 协议电机
 */
typedef enum {
    SERVO_CLOCKWISE = -1,   /* Servomotor always turn clockwisely */
    SERVO_NEAREST = 0,      /* Servomotor turn in direction that make movement minimum */
    SERVO_ANTICLOCKWISE = 1 /* Servomotor always turn anticlockwisely */
} servo_mode_t;

/**
 * @brief 伺服电机旋转的状态，用于 DJI 的 CAN 协议电机
 */
typedef enum {
    TURNING_CLOCKWISE = -1,   /* Servomotor is turning clockwisely         */
    INPUT_REJECT = 0,         /* Servomotor rejecting current target input */
    TURNING_ANTICLOCKWISE = 1 /* Servomotor is turning anticlockwisely     */
} servo_status_t;

/**
 * @brief DJI 减速电机的减速比例，具体数值请参考电机说明书
 */
#define M3508P19_RATIO (3591.0 / 187) /* Transmission ratio of M3508P19 */
#define M2006P36_RATIO 36             /* Transmission ratio of M2006P36 */

typedef struct {
    servo_mode_t mode; /* turning mode of servomotor, refer to type servo_mode_t */
    float speed;       /* motor shaft turning speed                              */
} servo_jam_t;

class ServoMotor;  // declare first for jam_callback_t to have correct param type

/**
 * @brief 堵转回调函数模板
 */
typedef void (*jam_callback_t)(ServoMotor* servo, const servo_jam_t data);

/**
 * @brief 伺服电机的初始化结构体
 */
typedef struct {
    DjiMotorBase* motor;      /* motor instance to be wrapped as a servomotor      */
    float max_speed;          /* desired turning speed of motor shaft, in [rad/s]  */
    float max_acceleration;   /* desired acceleration of motor shaft, in [rad/s^2] */
    float transmission_ratio; /* transmission ratio of motor */
    float* omega_pid_param;   /* pid parameter used to control speed of motor   */
    float max_iout;
    float max_out;
    float* hold_pid_param;
    float hold_max_iout;
    float hold_max_out;
} servo_t;

/**
 * @brief 伺服电机的包装类，用于精确控制电机的角度，可以用于带有外部减速箱的电机
 *
 * @note 这是一个计算类，它计算电机的输出以达到目标角度，但它不直接控制电机转动
 */
class ServoMotor {
  public:
    /**
     * @brief 基础构造函数
     *
     * @param servo         初始化结构体，参考 servo_t
     * @param proximity_in  电机进入保持状态的临界角度差
     * @param proximity_out 电机退出保持状态的临界角度差
     *
     * @note proximity_out 应该大于 proximity_in
     */
    ServoMotor(servo_t data, float align_angle = -1, float proximity_in = 0.05,
               float proximity_out = 0.15);

    /**
     * @brief 设置电机的目标角度，如果上一个目标角度没有达到，那么这个函数将不会有任何效果
     *
     * @note 如果电机没有进入保持状态，那么这个函数将不会有任何效果，除非 override 为 true
     *
     * @param target   电机的目标角度，单位为 [rad]
     * @param override 如果为 true，那么无论电机是否进入保持状态，都会覆盖当前的目标角度
     *
     * @return 电机的当前旋转模式
     */
    servo_status_t SetTarget(const float target, bool override = false);

    /**
     * @brief 设置电机的最大旋转速度
     *
     * @note 应该始终为正数，负数将被忽略
     *
     * @param max_speed 电机的最大旋转速度，单位为 [rad/s]
     */
    void SetMaxSpeed(const float max_speed);

    /**
     * @brief 设置电机的最大旋转加速度
     *
     * @note 应该始终为正数，负数将被忽略
     *
     * @param max_acceleration 电机的最大旋转加速度，单位为 [rad/s^2]
     */
    void SetMaxAcceleration(const float max_acceleration);

    /**
     * @brief 通过当前配置，计算电机的实际输出值
     *
     * @note 这个函数不会直接控制电机，它只会计算电机的输出值
     */
    void CalcOutput();

    void Hold(bool override = false);

    /**
     * @brief 检测电机是否进入锁定状态
     *
     * @return true  电机进入锁定状态
     * @return false 电机没有进入锁定状态
     */
    bool Holding() const;

    /**
     * @brief 获取电机当前的目标角度，单位为 [rad]
     *
     * @return 电机当前的目标角度，范围为 [0, 2PI]
     */
    float GetTarget() const;

    /**
     * @brief 注册电机的堵转回调函数
     *
     * @note
     * 堵转检测使用一个移动窗口，它使用一个大小为 detect_period 的循环缓冲区来存储历史输入，并计算输入的滚动平均值。
     *      每当输入的平均值大于 effect_threshold * 32768（电机可以接受的最大命令）时，堵转回调函数将被触发一次。
     *      回调函数只会在滚动平均值从低到高越过阈值时触发一次。
     *      对于标准的堵转回调函数，请参考示例 motor_m3508_antijam
     *
     * @param callback         要注册的回调函数
     * @param effort_threshold 电机被判定为堵转的阈值，范围为 (0, 1)
     * @param detect_period    检测窗口长度
     */
    void RegisterJamCallback(jam_callback_t callback, float effort_threshold,
                             uint8_t detect_period = 50);

    /**
     * @brief 打印电机数据
     */
    void PrintData() const;

    /**
     * @brief 获取电机的角度，单位为 [rad]
     *
     * @return 电机的弧度角度，范围为 [0, 2PI]
     */
    float GetTheta() const;

    /**
     * @brief 获取电机的角度与目标角度的角度差，单位为 [rad]
     *
     * @param target 目标角度，单位为 [rad]
     *
     * @return 与目标角度的弧度角度差，范围为 [-PI, PI]
     */
    float GetThetaDelta(const float target) const;

    /**
     * @brief 获取电机的角速度，单位为 [rad / s]
     *
     * @return 电机的角速度
     */
    float GetOmega() const;

    /**
     * @brief 获取电机的角速度与目标角速度的角速度差，单位为 [rad / s]
     *
     * @param target 目标角速度，单位为 [rad / s]
     *
     * @return 与目标角速度的角速度差
     */
    float GetOmegaDelta(const float target) const;

    /**
     * @brief 更新电机的反馈数据
     * @note 仅在 CAN 回调函数中使用，不要在其他地方调用
     *
     * @param data[]  原始数据
     */
    void UpdateData(const uint8_t data[]);

    friend class SteeringMotor;

  private:
    // refer to servo_t for details
    DjiMotorBase* motor_;
    float max_speed_;
    float max_acceleration_;
    float transmission_ratio_;
    float proximity_in_;
    float proximity_out_;

    // angle control
    bool hold_; /* true if motor is holding now, otherwise moving now */
    uint64_t start_time_;
    float target_angle_; /* desired target angle, range between [0, 2PI] in [rad] */
    float align_angle_;  /* motor angle when a instance of this class is created
                            with that motor    */
    float motor_angle_;  /* current motor angle in [rad], with align_angle
                            subtracted               */
    float offset_angle_; /* cumulative offset angle of motor shaft, range between
                            [0, 2PI] in [rad] */
    float servo_angle_;  /* current angle of motor shaft, range between [0, 2PI]
                            in  [rad]           */
    float cumulated_angle_;

    // jam detection
    jam_callback_t jam_callback_; /* callback function that will be invoked if motor jammed */
    int detect_head_;             /* circular buffer current head             */
    int detect_period_;           /* circular buffer length           */
    int detect_total_;            /* rolling sum of motor inputs            */
    int jam_threshold_;           /* threshold for rolling sum for the motor to be
                                     considered as jammed */
    int16_t* detect_buf_;         /* circular buffer         */

    // pid controllers
    control::ConstrainedPID omega_pid_; /* pid for controlling speed of motor */
    control::ConstrainedPID hold_pid_;

    // edge detectors
    FloatEdgeDetector* inner_wrap_detector_; /* detect motor motion across encoder boarder */
    FloatEdgeDetector* outer_wrap_detector_; /* detect motor motion across encoder boarder */
    BoolEdgeDetector* hold_detector_;        /* detect motor is in mode toggling, reset
                                                pid accordingly  */
    BoolEdgeDetector* jam_detector_;         /* detect motor jam toggling, call jam
                                                callback accordingly */
};

typedef bool (*align_detect_t)(void);

/**
 * @brief 舵轮用转向电机的初始化结构体
 */
typedef struct {
    DjiMotorBase* motor; /* motor instance to be wrapped as a servomotor      */
    float max_speed;     /* desired turning speed of motor shaft, in [rad/s]  */
    float test_speed;
    float max_acceleration;   /* desired acceleration of motor shaft, in [rad/s^2] */
    float transmission_ratio; /* transmission ratio of motor */
    float offset_angle;
    float* omega_pid_param; /* pid parameter used to control speed of motor */
    float max_iout;
    float max_out;
    align_detect_t align_detect_func;
    float calibrate_offset;
} steering_t;

/**
 * @brief 舵轮用转向电机的包装类，用于精确控制电机的角度，可以用于带有外部减速箱的电机
 *
 * @note 这个类现在还处于测试阶段，如果没有专业知识，请不要使用
 */
class SteeringMotor {
  public:
    SteeringMotor(steering_t data);
    float GetRawTheta() const;
    /**
     * @brief print out motor data
     */
    void PrintData() const;
    void TurnRelative(float angle);
    void TurnAbsolute(float angle);
    bool AlignUpdate();
    void Update();

  private:
    ServoMotor* servo_;

    float test_speed_;
    align_detect_t align_detect_func;
    float calibrate_offset;

    float align_angle_;
    BoolEdgeDetector* align_detector;
    bool align_complete_;
};

/**
 * @brief 飞轮电机的初始化结构体
 */
typedef struct {
    DjiMotorBase* motor;    /* motor instance to be wrapped as a flywheel      */
    float max_speed;        /* desired turning speed of motor shaft, in [rad/s]  */
    float* omega_pid_param; /* pid parameter used to control speed of motor   */
    bool is_inverted;
} flywheel_t;

/**
 * @brief 飞轮电机的包装类
 *
 * @note 飞轮电机是一个特殊的类，使用 PID 控制器控制电机的角速度，使速度达到目标值且恒定
 * 通常用来控制摩擦轮的转速
 */
class FlyWheelMotor {
  public:
    /**
     * @brief 基础构造函数
     * @param data 飞轮电机的初始化结构体，参考 flywheel_t
     */
    FlyWheelMotor(flywheel_t data);
    /**
     * @brief 设置电机的旋转速度
     * @param speed 电机的目标旋转速度，单位为 [rad/s]
     */
    void SetSpeed(float speed);
    /**
     * @brief 计算电机的输出
     *
     * @note 这个函数不会直接控制电机，它只会计算电机的输出值
     */
    void CalcOutput();
    /**
     * @brief 获取电机的目标速度，单位为 [rad / s]
     *
     * @return 电机的目标速度
     */
    float GetTarget() const;
    /**
     * @brief 打印电机数据
     */
    void PrintData() const;

    /**
     * @brief 获取电机的角度，单位为 [rad]
     *
     * @return 电机的弧度角度，范围为 [0, 2PI]
     */
    float GetTheta() const;

    /**
     * @brief 获取电机的角度与目标角度的角度差，单位为 [rad]
     *
     * @param target 目标角度，单位为 [rad]
     *
     * @return 与目标角度的弧度角度差，范围为 [-PI, PI]
     */
    float GetThetaDelta(const float target) const;

    /**
     * @brief 获取电机的角速度，单位为 [rad / s]
     *
     * @return 电机的角速度
     */
    float GetOmega() const;

    /**
     * @brief 获取电机的角速度与目标角速度的角速度差，单位为 [rad / s]
     *
     * @param target 目标角速度，单位为 [rad / s]
     *
     * @return 与目标角速度的角速度差
     */
    float GetOmegaDelta(const float target) const;

    /**
     * @brief 更新电机的反馈数据
     *
     * @note 仅在 CAN 回调函数中使用，不要在其他地方调用
     *
     * @param data[]  原始数据
     */
    void UpdateData(const uint8_t data[]);

  private:
    DjiMotorBase* motor_;
    bool is_inverted_;
    float max_speed_;
    float target_speed_;
    control::PIDController omega_pid_;
};
}  // namespace driver
