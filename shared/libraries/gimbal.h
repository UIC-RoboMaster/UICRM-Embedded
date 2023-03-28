#pragma once

#include "can.h"
#include "controller.h"
#include "motor.h"
#include "utils.h"

namespace control {

    /**
     * @brief gimbal models
     */
    /**
     * @brief 云台类型
     */
    typedef enum { GIMBAL_FORTRESS, GIMBAL_SENTRY, GIMBAL_STEERING } gimbal_model_t;

    /**
     * @brief offset, max, and proximity angles of different gimbals
     * @note except for proximity is determined by user, these should be obtained
     * by reading encoder values through uart/gdb
     */
    /**
     * @brief 云台相关常数
     * @note 除了proximity由用户确定，其他应该通过读取编码器值来获取
     */
    typedef struct {
        float pitch_offset_; /* 俯仰偏移角（枪口位于垂直中心时的角度） */
        float yaw_offset_;   /* 偏航偏移角（枪口位于水平中心时的角度） */
        float pitch_max_;    /* 云台最大俯仰角                    */
        float yaw_max_;      /* 云台最大偏航角                    */
    } gimbal_data_t;

    /**
     * @brief structure used when gimbal instance is initialized
     */
    /**
     * @brief 云台初始化结构体
     */
    typedef struct {
        MotorCANBase* pitch_motor; /* pitch motor instance */
        MotorCANBase* yaw_motor;   /* yaw motor instance   */
        gimbal_model_t model;      /* gimbal model         */
    } gimbal_t;

    /**
     * @brief wrapper class for gimbal
     */
    /**
     * @brief 云台类
     */
    class Gimbal {
      public:
        /**
         * @brief constructor for gimbal
         *
         * @param gimbal structure that used to initialize gimbal, refer to type
         * gimbal_t
         */
        /**
         * @brief 构造函数
         *
         * @param gimbal 云台初始化结构体
         */
        Gimbal(gimbal_t gimbal);

        /**
         * @brief destructor for gimbal
         */
        /**
         * @brief 析构函数
         */
        ~Gimbal();

        /**
         * @brief get gimbal related constants
         *
         * @return refer to gimbal_data_t
         */
        /**
         * @brief 获取云台相关常数
         * @return 云台常数结构体
         */
        gimbal_data_t* GetData();

        /**
         * @brief calculate the output of the motors under current configuration
         * @note does not command the motor immediately
         */
        /**
         * @brief 计算电机输出
         * @note 会调用电机的SetSpeed()函数以设置电机速度
         * @note 不会立即命令电机
         */
        void Update();

        /**
         * @brief set motors to point to a new orientation
         *
         * @param new_pitch new pitch angled
         * @param new_yaw   new yaw angled
         */
        /**
         * @brief 设置云台电机绝对角度
         *
         * @param new_pitch 新绝对俯仰角
         * @param new_yaw   新绝对偏航角
         */
        void TargetAbs(float new_pitch, float new_yaw);

        /**
         * @brief set motors to point to a new orientation
         *
         * @param new_pitch new pitch angled
         * @param new_yaw   new yaw angled
         */
        /**
         * @brief 设置俯仰轴电机相对角度，偏航轴电机绝对角度
         *
         * @param new_pitch 新相对俯仰角
         * @param new_yaw   新绝对偏航角
         */
        void TargetAbsYawRelPitch(float new_pitch, float new_yaw);

        /**
         * @brief set motors to point to a new orientation
         *
         * @param new_pitch new pitch angled
         * @param new_yaw   new yaw angled
         */
        /**
         * @brief 设置云台相对角度（变化角度）
         *
         * @param new_pitch 新相对俯仰角
         * @param new_yaw   新相对偏航角
         */
        void TargetRel(float new_pitch, float new_yaw);

        /**
         * @brief update the offset of the gimbal
         *
         * @param pitch_offset new pitch offset
         * @param yaw_offset   new yaw offset
         */
        /**
         * @brief 更新云台偏移角
         * @param pitch_offset 新的俯仰偏移角
         * @param yaw_offset  新的偏航偏移角
         */
        void UpdateOffset(float pitch_offset, float yaw_offset);

      private:
        // acquired from user
        MotorCANBase* pitch_motor_ = nullptr;
        MotorCANBase* yaw_motor_ = nullptr;
        gimbal_model_t model_;

        // pitch and yaw constants
        gimbal_data_t data_;

        // pitch and yaw pid
        float* pitch_theta_pid_param_ =
            nullptr; /* pid param that used to control pitch motor when moving  */
        float* pitch_omega_pid_param_ =
            nullptr; /* pid param that used to control pitch motor when holding */
        float* yaw_theta_pid_param_ =
            nullptr; /* pid param that used to control yaw motor when moving */
        float* yaw_omega_pid_param_ =
            nullptr; /* pid param that used to control yaw motor when holding */
        ConstrainedPID* pitch_theta_pid_ = nullptr; /* pitch theta pid */
        ConstrainedPID* pitch_omega_pid_ = nullptr; /* pitch omega pid */
        ConstrainedPID* yaw_theta_pid_ = nullptr;   /* yaw theta pid   */
        ConstrainedPID* yaw_omega_pid_ = nullptr;   /* yaw omega pid   */

        // pitch and yaw angle
        float pitch_angle_; /* current gimbal pitch angle */
        float yaw_angle_;   /* current gimbal yaw angle   */

        // state detectors
        BoolEdgeDetector pitch_detector_; /* pitch pid mode toggle detector */
        BoolEdgeDetector yaw_detector_;   /* yaw pid mode toggle detector   */
    };

}  // namespace control
