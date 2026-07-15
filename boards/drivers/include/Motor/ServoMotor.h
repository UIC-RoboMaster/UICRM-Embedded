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
#include "pid.h"

namespace driver {

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
        ServoMotor(servo_t data, float align_angle = -1, float proximity_in = 0.05, float proximity_out = 0.15);

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
        void RegisterJamCallback(jam_callback_t callback, float effort_threshold, uint8_t detect_period = 50);

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

}  // namespace driver
