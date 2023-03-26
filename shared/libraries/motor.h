#pragma once

#include "bsp_can.h"
#include "bsp_pwm.h"
#include "controller.h"
#include "utils.h"

namespace control {

    constexpr int MOTOR_RANGE = 30000;  // TODO: 32767 or 30000?

    int16_t ClipMotorRange(float output);

    /**
     * @brief 所有电机的基类
     */
    class MotorBase {
      public:
        MotorBase() : output_(0) {
        }

        virtual ~MotorBase() {
        }

        /**
         * @brief 设置电机的输出电流值
         * @param val 16bit电流值
         */
        virtual void SetOutput(int16_t val) {
            output_ = val;
        }

      protected:
        int16_t output_;  // 电机当前设置的电流值
    };

    /**
     * @brief CAN协议电机的基类
     */
    class MotorCANBase : public MotorBase {
      public:
        /**
         * @brief base constructor
         *
         * @param can    CAN instance
         * @param rx_id  CAN rx id
         */
        MotorCANBase(bsp::CAN* can, uint16_t rx_id);

        /**
         * @brief 解析电调协议，保存电机发送的数据
         * @note only used in CAN callback, do not call elsewhere
         *
         * @param data[]  raw data bytes
         */
        virtual void UpdateData(const uint8_t data[]) = 0;

        /**
         * @brief print out motor data
         */
        virtual void PrintData() const = 0;

        /**
         * @brief 获得电机的角度，单位为 [rad]
         * @return 角度，范围为 [0, 2PI]，单位为 [rad]
         */
        virtual float GetTheta() const;

        /**
         * @brief 获得电机角度差 (target - actual)
         * @param target  目标角度, 单位为 [rad]
         * @return 角度差，范围为 [-PI, PI]，单位为 [rad]
         */
        virtual float GetThetaDelta(const float target) const;

        /**
         * @brief 获得电机的角速度
         * @return 角速度，单位为[rad / s]
         */
        virtual float GetOmega() const;

        /**
         * @brief 获得电机的角速度差 (target - actual)
         * @param target  目标角速度, 单位为 [rad / s]
         * @return 角速度差，单位为 [rad / s]
         */
        virtual float GetOmegaDelta(const float target) const;

        /**
         * @brief 获得电机的电流
         * @return 16bit电流值
         */
        virtual int16_t GetCurr() const;

        virtual uint16_t GetTemp() const;

        /**
         * @brief 发送电机设置的输出电流到CAN缓存
         *
         * @param motors[]    CAN协议电机指针
         * @param num_motors  电机数量
         *
         * @note static函数，将一组最多四个电机的输出电流值打包成一个32bit的数据一起发送到CAN
         */
        static void TransmitOutput(MotorCANBase* motors[], uint8_t num_motors);

        /**
         * @brief set ServoMotor as friend of MotorCANBase since they need to use
         *        many of the private parameters of MotorCANBase.
         */
        friend class ServoMotor;

        volatile bool connection_flag_ = false;  // 电机已连接

      protected:
        volatile float theta_;  // 角度
        volatile float omega_;  // 角速度

      private:
        bsp::CAN* can_;   // 电机can接口
        uint16_t rx_id_;  // 电调rx_id
        uint16_t tx_id_;  // 电调tx_id
    };

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
    };

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
    };

    /**
     * @brief DJI 6020 motor class
     */
    class Motor6020 : public MotorCANBase {
      public:
        /* constructor wrapper over MotorCANBase */
        Motor6020(bsp::CAN* can, uint16_t rx_id);

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
    };

    /**
     * @brief DJI 6623 motor class
     */
    class Motor6623 : public MotorCANBase {
      public:
        /* constructor wrapper over MotorCANBase */
        Motor6623(bsp::CAN* can, uint16_t rx_id);

        /* implements data update callback */
        void UpdateData(const uint8_t data[]) override final;

        /* implements data printout */
        void PrintData() const override final;

        /* override base implementation with max current protection */
        void SetOutput(int16_t val) override final;

        /* override default implementation with not implemented */
        float GetOmega() const override final;

        float GetOmegaDelta(const float target) const override final;

      private:
        volatile int16_t raw_current_get_ = 0;
        volatile int16_t raw_current_set_ = 0;

        static const int16_t CURRENT_CORRECTION = -1;  // current direction is reversed
    };

    /**
     * @brief PWM协议电机基类
     */
    class MotorPWMBase : public MotorBase {
      public:
        /**
         * @brief constructor
         *
         * @param htim           HAL timer handle
         * @param channel        HAL timer channel, from [0, 4)
         * @param clock_freq     clock frequency associated with the timer, in [Hz]
         * @param output_freq    desired output frequency, in [Hz]
         * @param idle_throttle  idling pulse width, in [us]
         *
         * @note M3508 have idle_throttle about 1500, snail have idle_throttle about
         * 1100
         */
        MotorPWMBase(TIM_HandleTypeDef* htim, uint8_t channel, uint32_t clock_freq,
                     uint32_t output_freq, uint32_t idle_throttle);

        /**
         * @brief set and transmit output
         *
         * @param val offset value with respect to the idle throttle pulse width, in
         * [us]
         */
        virtual void SetOutput(int16_t val) override;

      private:
        bsp::PWM pwm_;
        uint32_t idle_throttle_;
    };

    /**
     * @brief DJI snail 2305 motor class
     */
    class Motor2305 : public MotorPWMBase {
      public:
        /* override base implementation with max current protection */
        void SetOutput(int16_t val) override final;
    };

    /**
     * @brief servomotor turning mode
     * @note the turning direction is determined as if user is facing the motor, may
     * subject to change depending on motor type
     */
    typedef enum {
        SERVO_CLOCKWISE = -1,   /* Servomotor always turn clockwisely */
        SERVO_NEAREST = 0,      /* Servomotor turn in direction that make movement minimum */
        SERVO_ANTICLOCKWISE = 1 /* Servomotor always turn anticlockwisely */
    } servo_mode_t;

    /**
     * @brief 伺服电机旋转方向
     * @note the turning direction is determined as if user is facing the motor, may
     * subject to change depending on motor type
     */
    typedef enum {
        TURNING_CLOCKWISE = -1,   /* 正在顺时针旋转     */
        INPUT_REJECT = 0,         /* 忽略此次设置操作   */
        TURNING_ANTICLOCKWISE = 1 /* 正在逆时针旋转     */
    } servo_status_t;

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

    class ServoMotor;  // declare first for jam_callback_t to have correct param type
                       /**
                        * @brief jam callback template
                        */
    typedef void (*jam_callback_t)(ServoMotor* servo, const servo_jam_t data);

    /**
     * @brief 伺服电机参数，初始化伺服电机实例时使用
     */
    typedef struct {
        MotorCANBase* motor;      /* motor instance to be wrapped as a servomotor      */
        float max_speed;          /* desired turning speed of motor shaft, in [rad/s]  */
        float max_acceleration;   /* desired acceleration of motor shaft, in [rad/s^2] */
        float transmission_ratio; /* transmission ratio of motor */
        float* omega_pid_param;   /* pid parameter used to control speed of motor   */
        float max_iout;
        float max_out;
    } servo_t;

    /**
     * @brief 包装类，有齿轮箱时精确控制轴角度
     * @note 这是一个计算类，用于计算所需的输出，但它不会直接命令电机转动
     */
    class ServoMotor {
      public:
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
         * @brief 为伺服电机设定下一个目标
         * @note 通过是否进入holding状态判断是否到达上一目标。
         * @note 若上一设定目标没有到达，则忽略此次设定，除非override=true
         *
         * @param target   next target for the motor in [rad]
         * @param override if true, override current target even if motor is not
         * holding right now
         * @return current turning mode of motor
         */
        servo_status_t SetTarget(const float target, bool override = false);

        /**
         * @brief 设置旋转时的速度
         * @note should always be positive, negative inputs will be ignored
         *
         * @param max_speed speed of desired motor shaft turning speed, in [rad/s]
         */
        void SetMaxSpeed(const float max_speed);

        /**
         * @brief 设置旋转时的加速度
         * @note should always be positive, negative inputs will be ignored
         *
         * @param max_acceleration speed of desired motor shaft turning speed, in
         * [rad/s]
         */
        void SetMaxAcceleration(const float max_acceleration);

        /**
         * @note 不发送电机指令，仅计算电流
         */
        void CalcOutput();

        /**
         * @brief 电机是否在holding状态
         *
         * @return true  the motor is holding (i.e. not turning)
         * @return false the motor is not holding (i.e. turning)
         */
        bool Holding() const;

        /**
         * @brief get current servomotor target, in [rad]
         *
         * @return current target angle, range between [0, 2PI]
         */
        float GetTarget() const;

        /**
         * @brief register a callback function that would be called if motor is jammed
         * @note Jam detection uses a moving window across inputs to the motor. It
         * uses a circular buffer of size detect_period to store history inputs and
         * calculates a rolling average of the inputs. Everytime the average of inputs
         * is greater than effect_threshold * 32768(maximum command a motor can
         * accept), the jam callback function will be triggered once. The callback
         * will only be triggered once each time the rolling average cross the
         * threshold from lower to higher. For a standard jam callback function, refer
         * to example motor_m3508_antijam
         *
         * @param callback         callback function to be registered
         * @param effort_threshold threshold for motor to be determined as jammed,
         * ranged between (0, 1)
         * @param detect_period    detection window length
         */
        void RegisterJamCallback(jam_callback_t callback, float effort_threshold,
                                 uint8_t detect_period = 50);

        /**
         * @brief print out motor data
         */
        void PrintData() const;

        /**
         * @brief 获得电机的角度，单位为 [rad]
         * @return 角度，范围为 [0, 2PI]，单位为 [rad]
         */
        float GetTheta() const;

        /**
         * @brief 获得电机角度差 (target - actual)
         * @param target  目标角度, 单位为 [rad]
         * @return 角度差，范围为 [-PI, PI]，单位为 [rad]
         */
        float GetThetaDelta(const float target) const;

        /**
         * @brief 获得电机的角速度
         * @return 角速度，单位为[rad / s]
         */
        float GetOmega() const;

        /**
         * @brief 获得电机的角速度差 (target - actual)
         * @param target  目标角速度, 单位为 [rad / s]
         * @return 角速度差，单位为 [rad / s]
         */
        float GetOmegaDelta(const float target) const;

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
        uint32_t start_time_;
        float target_angle_; /* desired target angle, range between [0, 2PI] in [rad] */
        float align_angle_;  /* motor angle when a instance of this class is created
                          with that motor    */
        float motor_angle_;  /* current motor angle in [rad], with align_angle
                          subtracted               */
        float offset_angle_; /* cumulative offset angle of motor shaft, range between
                          [0, 2PI] in [rad] */
        float servo_angle_;  /* current angle of motor shaft, range between [0, 2PI] in
                          [rad]           */
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
        ConstrainedPID omega_pid_; /* pid for controlling speed of motor */

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

} /* namespace control */
