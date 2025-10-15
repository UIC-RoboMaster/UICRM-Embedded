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

#include "MotorBase.h"
#include "bsp_can.h"
#include "bsp_thread.h"
#include "connection_driver.h"
#include "pid.h"
#include "utils.h"

#define M3508P19_MAX_OUTPUT 12000.0f

namespace driver {

    /**
     * @brief 带有CAN通信的DJI通用电机的标准接口
     */
    class MotorCANBase : public MotorBase, public ConnectionDriver {
      public:
        enum motor_mode {
            // 未使用
            NONE = 0x00,
            // 未使用
            CURRENT = 0x01,
            // 启用角度环控制
            OMEGA = 0x02,
            // 启用速度环控制
            THETA = 0x04,
            // 反转电机方向
            INVERTED = 0x40,
            // ABSOLUTE模式下，认为输出轴只有一圈。电机输出轴角度不会累计，被限制在[0,
            // 2PI]之间，如果目标在相反的半圈，则从另一侧绕过去
            ABSOLUTE = 0x80,
        };

        /**
         * @brief 堵转回调函数模板
         */
        typedef void (*callback_t)(void* instance);

        /**
         * @brief 基础构造函数
         * @param can    CAN对象
         * @param rx_id  电机使用的CAN接收ID，参考电机的说明书
         */
        MotorCANBase(bsp::CAN* can, uint16_t rx_id, uint16_t tx_id = 0x00);

        static void SetFrequency(uint32_t freq = 1000);

        /**
         * @brief 更新电机的反馈数据
         * @note 仅在CAN回调函数中使用，不要在其他地方调用
         * @param data[]  原始数据
         */
        virtual void UpdateData(const uint8_t data[]);

        /**
         * @brief 打印电机数据
         */
        virtual void PrintData() const = 0;

        /**
         * @brief 电机编码器角度，格式为[rad]，范围为[0, 2PI]
         */
        virtual float GetTheta() const;

        /**
         * @brief 电机编码器角速度，格式为[rad / s]
         */
        virtual float GetOmega() const;

        /**
         * @brief 获得电机的累计输出轴角度（经过变速箱且编码器在变速箱之前），格式为[rad]
         * @note ABSOLUTE模式下，认为输出轴只有一圈，输出轴角度不会累计，被限制在[0, 2PI]之间
         */
        virtual float GetOutputShaftTheta() const;

        /**
         * @brief 获得电机的输出轴角速度（经过变速箱且编码器在变速箱之前），格式为[rad / s]
         */
        virtual float GetOutputShaftOmega() const;

        /**
         * @brief 获得电机的编码器角度与目标角度的角度差，格式为[rad]
         * @param target 目标角度，格式为[rad]
         * @return 与目标角度的弧度角度差，范围为[-PI, PI]
         */
        virtual float GetThetaDelta(const float target) const;

        /**
         * @brief 获得电机的编码器角速度与目标角速度的角速度差，格式为[rad / s]
         * @param target 目标角速度，格式为[rad / s]
         * @return 与目标角速度的角速度差
         */
        virtual float GetOmegaDelta(const float target) const;

        /**
         * @return 电调反馈的转矩电流，单位为[mA]
         */
        virtual int16_t GetCurr() const;

        /**
         * @return 电调反馈的温度，单位为[℃]
         */
        virtual uint16_t GetTemp() const;

        /**
         * @brief 通过电机的pid控制器计算电机的输出
         * @note 本函数会在电机输出进程中按照所设定的频率被自动调用，正常情况下请勿手动调用
         */
        virtual void CalcOutput();

        /**
         * @brief 设置目标
         * @param target 设置输出轴的目标：角度[RAD]、累计角度[RAD]、角速度[RAD/S]（取决于模式）
         * @param override 电机为角度控制模式下，还未达到之前的目标时，是否覆盖旧的目标
         */
        virtual void SetTarget(float target, bool override = true);

        /**
         * @brief 读取上一次设置的目标值
         * @return 输出轴的目标：角度[RAD]、累计角度[RAD]、角速度[RAD/S]（取决于模式）
         */
        virtual float GetTarget() const;

        /**
         * @brief 设置电机的PID
         * @param pid_init pid的初始化参数
         * @param mode 所需要设置的pid的环，一般是速度环或者角度环
         */
        virtual void ReInitPID(control::ConstrainedPID::PID_Init_t pid_init, uint8_t mode);

        /**
         * @brief 获取电机PID数值
         */
        virtual control::ConstrainedPID::PID_State_t GetPIDState(uint8_t mode) const;

        /**
         * @brief
         * 设置电机的工作模式，工作模式由若干个bool值组成，请参考电机模式的定义，启动多个模式的情况需要使用或运算
         * @refitem motor_mode
         * @param mode 电机的工作模式
         */
        void SetMode(uint8_t mode);

        /**
         * @brief 设置电机的减速箱比例
         * @param ratio 电机的减速箱比例
         */
        void SetTransmissionRatio(float ratio);

        /**
         * @brief 设置ServoMotor为MotorCANBase的友元，因为它们需要使用MotorCANBase的许多私有参数。
         */
        friend class ServoMotor;

        void Enable();
        void Disable();
        bool IsEnable() const;

        /**
         * @brief 设置堵转回调函数
         */
        void RegisterErrorCallback(callback_t callback, void* instance);

        /**
         * @brief 处理堵转回调函数
         * @param instance 关联的电机实例
         * @param type pid的故障类型
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
         * @param override 应为true
         */
        void Hold(bool override = true);

        /**
         * @brief 在电机目标速度（角度环PID的输出）上加上一个偏移量
         * @note 用于实现前馈
         */
        void SetSpeedOffset(float offset);

      protected:
        volatile float theta_;  // 编码器提供的角度值，单位为[rad]
        volatile float omega_;  // 编码器提供的速度值，单位为[rad/s]

        volatile float output_shaft_theta_;  // 电机输出轴的累计角度，单位为[rad]
        volatile float output_shaft_omega_;  // 电机输出轴的速度，单位为[rad/s]

        bool enable_;

        // angle control
        volatile float power_on_angle_ = 0; /* 上电时的编码器角度，单位为[rad] */
        volatile float relative_angle_ = 0; /* 编码器相对于开机角度的角度，单位为[rad] */
        volatile float cumulated_turns_ =
            0; /* 编码器累计圈数，按照2*PI/ratio加减，累积到2*PI清零 */
        volatile float output_cumulated_turns_ = 0; /* 输出轴累计圈数，按照2*PI加减，单位为[rad]*/
        volatile float output_relative_angle_ =
            0; /* 输出轴在这一圈中的角度，单位为[rad]，范围为[0, 2PI] */

        FloatEdgeDetector* inner_wrap_detector_; /* detect motor motion across encoder boarder */
        FloatEdgeDetector* outer_wrap_detector_; /* detect motor motion across encoder boarder */

        float transmission_ratio_ = 1; /* 电机的减速比例 */

        float proximity_in_ = 0.05; /* 电机进入保持状态的临界角度差 */

        float proximity_out_ = 0.15; /* 电机退出保持状态的临界角度差 */

        bool holding_ = true; /* 角度模式下，电机是否达到目标 */
      private:
        bsp::CAN* can_;
        uint16_t rx_id_;
        uint16_t tx_id_;

        uint8_t mode_ = 0;
        control::ConstrainedPID omega_pid_;
        control::ConstrainedPID theta_pid_;
        // use to calculate the delta time between two received data packets
        uint32_t last_update_time_us_;
        //
        uint32_t motor_update_time_interval;

        // 目标，取决于电机的模式，可以是角度[RAD]、角速度[RAD/S]
        float target_;

        // 前馈中使用，在角度环输出的速度上加上一个偏移量
        float speed_offset_;

        callback_t error_callback_ = [](void* instance) { UNUSED(instance); };
        void* error_callback_instance_ = nullptr;

        /**
         * @brief 发送CAN消息以设置电机输出
         * @param motors[]    CAN电机指针数组
         * @param num_motors  要发送的电机数量
         */
        static void TransmitOutput(MotorCANBase* motors[], uint8_t num_motors);

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

        static uint16_t id_[10];
        static bsp::CAN* can_to_index_[10];
        static uint8_t group_cnt_;
        static MotorCANBase* motors_[10][4];
        static uint8_t motor_cnt_[10];
        static uint32_t delay_time;

        static callback_t pre_output_callback_;
        static void* pre_output_callback_instance_;
        static callback_t post_output_callback_;
        static void* post_output_callback_instance_;
    };

    /**
     * @brief DJI 2006电机的标准类
     */
    /**
     * @brief DJI 2006 motor class
     */
    class Motor2006 : public MotorCANBase {
      public:
        /* constructor wrapper over MotorCANBase */
        Motor2006(bsp::CAN* can, uint16_t rx_id);
        /* implements data update callback */
        void UpdateData(const uint8_t data[]) override final;
        /* implements data printout */
        void PrintData() const override final;
        /* override base implementation with max current protection */
        void SetOutput(int16_t val) override final;

        int16_t GetCurr() const override final;

      private:
        volatile int16_t raw_current_get_ = 0;
        static const int16_t MAX_OUT = 10000;
    };

    /**
     * @brief DJI 3508电机的标准类
     */
    /**
     * @brief DJI 3508 motor class
     */
    class Motor3508 : public MotorCANBase {
      public:
        /* constructor wrapper over MotorCANBase */
        Motor3508(bsp::CAN* can, uint16_t rx_id);
        /* implements data update callback */
        void UpdateData(const uint8_t data[]) override final;
        /* implements data printout */
        void PrintData() const override final;
        /* override base implementation with max current protection */
        void SetOutput(int16_t val) override final;

        int16_t GetCurr() const override final;

        uint16_t GetTemp() const override final;

      private:
        volatile int16_t raw_current_get_ = 0;
        volatile uint8_t raw_temperature_ = 0;
        static const int16_t MAX_OUT = 32767;
    };

    /**
     * @brief DJI 6020电机的标准类
     */
    /**
     * @brief DJI 6020 motor class
     */
    class Motor6020 : public MotorCANBase {
      public:
        /* constructor wrapper over MotorCANBase */
        Motor6020(bsp::CAN* can, uint16_t rx_id, uint16_t tx_id = 0x00);
        /* implements data update callback */
        void UpdateData(const uint8_t data[]) override final;
        /* implements data printout */
        void PrintData() const override final;
        /* override base implementation with max current protection */
        void SetOutput(int16_t val) override final;

        int16_t GetCurr() const override final;

        uint16_t GetTemp() const override final;

        void SetSpeedFilter(float ratio);

      private:
        volatile int16_t raw_current_get_ = 0;
        volatile uint8_t raw_temperature_ = 0;
        static const int16_t MAX_OUT = 25000;
        static const int16_t MAX_OUT_C = 16383;
        float input_speed_filter_ = 0.1;
    };

    /**
     * @brief DM4310电机的标准类
     */
    /**
     * @brief DM4310 motor class
     */
    class MotorDM4310 : public MotorCANBase {
      public:
        /* constructor wrapper over MotorCANBase */
        MotorDM4310(bsp::CAN* can, uint16_t rx_id, uint16_t tx_id);
        /* implements data update callback */
        void UpdateData(const uint8_t data[]) override final;
        /* implements data printout */
        void PrintData() const override final;
        /* override base implementation with max current protection */
        void SetOutput(int16_t val) override final;

        int16_t GetCurr() const override final;

        uint16_t GetTemp() const override final;

      private:
        volatile int16_t raw_current_get_ = 0;
        volatile uint8_t raw_temperature_ = 0;
        volatile uint8_t raw_temperature_esc_ = 0;
        static const int16_t MAX_OUT = 32767;
    };

    /**
     * @brief 伺服电机旋转模式，用于DJI的CAN协议电机
     *
     * @note 旋转方向是相对于用户面向电机的方向而言的，具体旋转方向取决于电机的类型
     *
     * @enum SERVO_CLOCKWISE      电机顺时针旋转
     * @enum SERVO_NEAREST        电机停止转动，锁定状态
     * @enum SERVO_ANTICLOCKWISE  电机逆时针旋转
     */
    /**
     * @brief servomotor turning mode
     * @note the turning direction is determined as if user is facing the motor,
     * may subject to change depending on motor type
     *
     * @enum SERVO_CLOCKWISE      motor turn clockwisely
     * @enum SERVO_NEAREST        motor stop turning, hold state
     * @enum SERVO_ANTICLOCKWISE  motor turn anticlockwisely
     */
    typedef enum {
        SERVO_CLOCKWISE = -1,   /* Servomotor always turn clockwisely */
        SERVO_NEAREST = 0,      /* Servomotor turn in direction that make movement minimum */
        SERVO_ANTICLOCKWISE = 1 /* Servomotor always turn anticlockwisely */
    } servo_mode_t;

    /**
     * @brief 伺服电机旋转的状态，用于DJI的CAN协议电机
     *
     * @note 旋转方向是相对于用户面向电机的方向而言的，具体旋转方向取决于电机的类型
     *
     * @enum TURNING_CLOCKWISE      电机顺时针旋转
     * @enum INPUT_REJECT           电机拒绝当前的输入
     * @enum TURNING_ANTICLOCKWISE  电机逆时针旋转
     */
    /**
     * @brief servomotor status
     * @note the turning direction is determined as if user is facing the motor,
     * may subject to change depending on motor type
     *
     * @enum TURNING_CLOCKWISE      motor turn clockwisely
     * @enum INPUT_REJECT           motor reject current target input
     * @enum TURNING_ANTICLOCKWISE  motor turn anticlockwisely
     */
    typedef enum {
        TURNING_CLOCKWISE = -1,   /* Servomotor is turning clockwisely         */
        INPUT_REJECT = 0,         /* Servomotor rejecting current target input */
        TURNING_ANTICLOCKWISE = 1 /* Servomotor is turning anticlockwisely     */
    } servo_status_t;

    /**
     * @brief DJI减速电机的减速比例，具体数值请参考电机说明书
     */
/**
 * @brief transmission ratios of DJI motors, reference to motor manuals for
 * more details
 */
#define M3508P19_RATIO (3591.0 / 187) /* Transmission ratio of M3508P19 */
#define M2006P36_RATIO 36             /* Transmission ratio of M2006P36 */

    typedef struct {
        servo_mode_t mode; /* turning mode of servomotor, refer to type servo_mode_t */
        float speed;       /* motor shaft turning speed                              */
    } servo_jam_t;

    class ServoMotor;  // declare first for jam_callback_t to have correct param
                       // type

    /**
     * @brief 堵转回调函数模板
     */
    /**
     * @brief jam callback template
     */
    typedef void (*jam_callback_t)(ServoMotor* servo, const servo_jam_t data);

    /**
     * @brief 伺服电机的初始化结构体
     */
    /**
     * @brief structure used when servomotor instance is initialized
     */
    typedef struct {
        MotorCANBase* motor;      /* motor instance to be wrapped as a servomotor      */
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
     *
     */
    /**
     * @brief wrapper class for motor to enable the motor shaft angle to be
     * precisely controlled with possible external gearbox present
     * @note this is a calculation class that calculate the motor output for
     * desired output, but it does not directly command a motor to turn.
     */
    class ServoMotor {
      public:
        /**
         * @brief 基础构造函数
         *
         * @param servo         初始化结构体，参考servo_t
         * @param proximity_in  电机进入保持状态的临界角度差
         * @param proximity_out 电机退出保持状态的临界角度差
         *
         * @note proximity_out应该大于proximity_in
         */
        /**
         * @brief base constructor
         *
         * @param servo         initialization struct, refer to type servo_t
         * @param proximity_in  critical difference angle for the motor to enter hold
         * state
         * @param proximity_out critical difference angle for the motor to exit hold
         * state
         *
         * @note proximity_out should be greater than proximity_in
         */
        ServoMotor(servo_t data, float align_angle = -1, float proximity_in = 0.05,
                   float proximity_out = 0.15);

        /**
         * @brief 设置电机的目标角度，如果上一个目标角度没有达到，那么这个函数将不会有任何效果
         *
         * @note 如果电机没有进入保持状态，那么这个函数将不会有任何效果，除非override为true
         *
         * @param target   电机的目标角度，单位为[rad]
         * @param override 如果为true，那么无论电机是否进入保持状态，都会覆盖当前的目标角度
         *
         * @return 电机的当前旋转模式
         */
        /**
         * @brief set next target for servomotor, will have no effect if last set
         * target has not been achieved
         * @note if motor is not holding, call to this function will have no effect
         * unless override is true
         *
         * @param target   next target for the motor in [rad]
         * @param override if true, override current target even if motor is not
         * holding right now
         * @return current turning mode of motor
         */
        servo_status_t SetTarget(const float target, bool override = false);

        /**
         * @brief 设置电机的最大旋转速度
         *
         * @note 应该始终为正数，负数将被忽略
         *
         * @param max_speed 电机的最大旋转速度，单位为[rad/s]
         */
        /**
         * @brief set turning speed of motor when moving
         *
         * @note should always be positive, negative inputs will be ignored
         *
         * @param max_speed speed of desired motor shaft turning speed, in [rad/s]
         */
        void SetMaxSpeed(const float max_speed);

        /**
         * @brief 设置电机的最大旋转加速度
         *
         * @note 应该始终为正数，负数将被忽略
         *
         * @param max_acceleration 电机的最大旋转加速度，单位为[rad/s^2]
         */
        /**
         * @brief set acceleration of motor when moving
         *
         * @note should always be positive, negative inputs will be ignored
         *
         * @param max_acceleration speed of desired motor shaft turning speed, in
         * [rad/s^2]
         */
        void SetMaxAcceleration(const float max_acceleration);

        /**
         * @brief 通过当前配置，计算电机的实际输出值
         *
         * @note 这个函数不会直接控制电机，它只会计算电机的输出值
         */
        /**
         * @brief calculate the output of the motors under current configuration
         * @note this function will not command the motor, it only calculate the
         * desired input
         */
        void CalcOutput();

        void Hold(bool override = false);

        /**
         * @brief 检测电机是否进入锁定状态
         *
         * @return true  电机进入锁定状态
         * @return false 电机没有进入锁定状态
         */
        /**
         * @brief if the motor is holding
         *
         * @return true  the motor is holding (i.e. not turning)
         * @return false the motor is not holding (i.e. turning)
         */
        bool Holding() const;

        /**
         * @brief 获取电机当前的目标角度，单位为[rad]
         *
         * @return 电机当前的目标角度，范围为[0, 2PI]
         */
        /**
         * @brief get current servomotor target, in [rad]
         *
         * @return current target angle, range between [0, 2PI]
         */
        float GetTarget() const;

        /**
         * @brief 注册电机的堵转回调函数
         *
         * @note
         * 堵转检测使用一个移动窗口，它使用一个大小为detect_period的循环缓冲区来存储历史输入，并计算输入的滚动平均值。
         *      每当输入的平均值大于effect_threshold *
         * 32768（电机可以接受的最大命令）时，堵转回调函数将被触发一次。
         *      回调函数只会在滚动平均值从低到高越过阈值时触发一次。
         *      对于标准的堵转回调函数，请参考示例motor_m3508_antijam
         *
         *      @param callback         要注册的回调函数
         *      @param effort_threshold 电机被判定为堵转的阈值，范围为(0, 1)
         *      @param detect_period    检测窗口长度
         */
        /**
         * @brief register a callback function that would be called if motor is
         * jammed
         * @note Jam detection uses a moving window across inputs to the motor. It
         * uses a circular buffer of size detect_period to store history inputs and
         * calculates a rolling average of the inputs. Everytime the average of
         * inputs is greater than effect_threshold * 32768(maximum command a motor
         * can accept), the jam callback function will be triggered once. The
         * callback will only be triggered once each time the rolling average cross
         * the threshold from lower to higher. For a standard jam callback function,
         * refer to example motor_m3508_antijam
         *
         * @param callback         callback function to be registered
         * @param effort_threshold threshold for motor to be determined as jammed,
         * ranged between (0, 1)
         * @param detect_period    detection window length
         */
        void RegisterJamCallback(jam_callback_t callback, float effort_threshold,
                                 uint8_t detect_period = 50);

        /**
         * @brief 打印电机数据
         */
        /**
         * @brief print out motor data
         */
        void PrintData() const;

        /**
         * @brief 获取电机的角度，单位为[rad]
         *
         * @return 电机的弧度角度，范围为[0, 2PI]
         */
        /**
         * @brief get motor angle, in [rad]
         *
         * @return radian angle, range between [0, 2PI]
         */
        float GetTheta() const;

        /**
         * @brief 获取电机的角度与目标角度的角度差，单位为[rad]
         *
         * @param target 目标角度，单位为[rad]
         *
         * @return 与目标角度的弧度角度差，范围为[-PI, PI]
         */
        /**
         * @brief get angle difference (target - actual), in [rad]
         *
         * @param target  target angle, in [rad]
         *
         * @return angle difference, range between [-PI, PI]
         */
        float GetThetaDelta(const float target) const;

        /**
         * @brief 获取电机的角速度，单位为[rad / s]
         *
         * @return 电机的角速度
         */
        /**
         * @brief get angular velocity, in [rad / s]
         *
         * @return angular velocity
         */
        float GetOmega() const;

        /**
         * @brief 获取电机的角速度与目标角速度的角速度差，单位为[rad / s]
         *
         * @param target 目标角速度，单位为[rad / s]
         *
         * @return 与目标角速度的角速度差
         */
        /**
         * @brief get angular velocity difference (target - actual), in [rad / s]
         *
         * @param target  target angular velocity, in [rad / s]
         *
         * @return difference angular velocity
         */
        float GetOmegaDelta(const float target) const;

        /**
         * @brief 更新电机的反馈数据
         * @note 仅在CAN回调函数中使用，不要在其他地方调用
         *
         * @param data[]  原始数据
         */
        /**
         * @brief update the current theta for the servomotor
         * @note only used in CAN callback, do not call elsewhere
         *
         * @param data[]  raw data bytes
         */
        void UpdateData(const uint8_t data[]);

        friend class SteeringMotor;

      private:
        // refer to servo_t for details
        MotorCANBase* motor_;
        float max_speed_;
        float max_acceleration_;
        float transmission_ratio_;
        float proximity_in_;
        float proximity_out_;

        // angle control
        bool hold_; /* true if motor is holding now, otherwise moving now */
        uint64_t start_time_;
        float target_angle_; /* desired target angle, range between [0, 2PI] in [rad]
                              */
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
        jam_callback_t jam_callback_; /* callback function that will be invoked if
                                         motor jammed */
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
    /**
     * @brief structure used when steering motor instance is initialized
     */
    typedef struct {
        MotorCANBase* motor; /* motor instance to be wrapped as a servomotor      */
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
     *
     */
    /**
     * @brief structure used when flywheel motor instance is initialized
     *
     */
    typedef struct {
        MotorCANBase* motor;    /* motor instance to be wrapped as a flywheel      */
        float max_speed;        /* desired turning speed of motor shaft, in [rad/s]  */
        float* omega_pid_param; /* pid parameter used to control speed of motor   */
        bool is_inverted;
    } flywheel_t;

    /**
     * @brief 飞轮电机的包装类
     *
     *
     * @note 飞轮电机是一个特殊的类，使用PID控制器控制电机的角速度，使速度达到目标值且恒定
     * 通常用来控制摩擦轮的转速
     */
    /**
     * @brief class for flywheel motor
     *
     * @note flywheel motor is a special class that uses PID controller to control the angular
     * velocity of motor, making the velocity reach the target and constant usually used to control
     * the speed of friction wheel
     */
    class FlyWheelMotor {
      public:
        /**
         * @brief 基础构造函数
         * @param data 飞轮电机的初始化结构体，参考flywheel_t
         */
        /**
         * @brief constructor for flywheel motor
         * @param data the data for flywheel
         */
        FlyWheelMotor(flywheel_t data);
        /**
         * @brief 设置电机的旋转速度
         * @param speed 电机的目标旋转速度，单位为[rad/s]
         */
        /**
         * @brief set the speed of flywheel motor
         * @param speed the target speed of flywheel motor in [rad/s]
         */
        void SetSpeed(float speed);
        /**
         * @brief 计算电机的输出
         *
         * @note 这个函数不会直接控制电机，它只会计算电机的输出值
         */
        /**
         * @brief calculate the output of flywheel motor
         *
         * @note this function will not command the motor, it only calculate the desired input
         */
        void CalcOutput();
        /**
         * @brief 获取电机的目标速度，单位为[rad / s]
         *
         * @return 电机的目标速度
         */
        /**
         * @brief get current target for flywheel, in [rad/s]
         *
         * @return current target speed
         */
        float GetTarget() const;
        /**
         * @brief 打印电机数据
         */
        /**
         * @brief print out motor data
         */
        void PrintData() const;

        /**
         * @brief 获取电机的角度，单位为[rad]
         *
         * @return 电机的弧度角度，范围为[0, 2PI]
         */
        /**
         * @brief get motor angle, in [rad]
         *
         * @return radian angle, range between [0, 2PI]
         */
        float GetTheta() const;

        /**
         * @brief 获取电机的角度与目标角度的角度差，单位为[rad]
         *
         * @param target 目标角度，单位为[rad]
         *
         * @return 与目标角度的弧度角度差，范围为[-PI, PI]
         */
        /**
         * @brief get angle difference (target - actual), in [rad]
         *
         * @param target  target angle, in [rad]
         *
         * @return angle difference, range between [-PI, PI]
         */
        float GetThetaDelta(const float target) const;

        /**
         * @brief 获取电机的角速度，单位为[rad / s]
         *
         * @return 电机的角速度
         */
        /**
         * @brief get angular velocity, in [rad / s]
         *
         * @return angular velocity
         */
        float GetOmega() const;

        /**
         * @brief 获取电机的角速度与目标角速度的角速度差，单位为[rad / s]
         *
         * @param target 目标角速度，单位为[rad / s]
         *
         * @return 与目标角速度的角速度差
         */
        /**
         * @brief get angular velocity difference (target - actual), in [rad / s]
         *
         * @param target  target angular velocity, in [rad / s]
         *
         * @return difference angular velocity
         */
        float GetOmegaDelta(const float target) const;

        /**
         * @brief 更新电机的反馈数据
         *
         * @note 仅在CAN回调函数中使用，不要在其他地方调用
         *
         * @param data[]  原始数据
         */
        /**
         * @brief update the current theta for the servomotor
         * @note only used in CAN callback, do not call elsewhere
         *
         * @param data[]  raw data bytes
         */
        void UpdateData(const uint8_t data[]);

      private:
        MotorCANBase* motor_;
        bool is_inverted_;
        float max_speed_;
        float target_speed_;
        control::PIDController omega_pid_;
    };
}  // namespace driver
