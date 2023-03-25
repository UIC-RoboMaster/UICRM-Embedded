#pragma once

#include "controller.h"
#include "motor.h"
#include "power_limit.h"

#define MAX_WHEEL_NUM 4

namespace control {

    /**
     * @brief 底盘类型
     */
    typedef enum {
        CHASSIS_MECANUM_WHEEL,  // 麦轮底盘
        CHASSIS_ONE_WHEEL,      // 单轮底盘
    } chassis_model_t;

    /**
     * @brief 底盘参数，初始化时用
     */
    typedef struct {
        MotorCANBase** motors;  /* motor instances of all chassis motors */
        chassis_model_t model; /* chassis model                         */
    } chassis_t;

    /**
     * @brief motor configs for four wheel vehicles
     */
    struct FourWheel {
        enum { front_left, front_right, back_left, back_right, motor_num };
    };

    struct OneWheel {
        enum { center, motor_num };
    };

    /**
     * @brief wrapper class for chassis
     */
    class Chassis {
      public:
        /**
         * @brief constructor for chassis
         *
         * @param chassis structure that used to initialize chassis, refer to type
         * chassis_t
         */
        Chassis(const chassis_t chassis);

        /**
         * @brief destructor for chassis
         */
        ~Chassis();

        /**
         * @brief 设置底盘速度并解算
         * @note 缓存在speeds_数组中
         *
         * @param x_speed chassis speed on x-direction
         * @param y_speed chassis speed on y-direction
         * @param turn_speed chassis clockwise turning speed
         */
        void SetSpeed(const float x_speed, const float y_speed = 0, const float turn_speed = 0);

        /**
         * @brief 输入当前功耗，计算电机16bit电流，发送给电机类
         * @param power_limit_on 是否开启功耗限制
         * @param power_limit 单位？
         * @param chassis_power 单位？
         * @param chassis_power_buffer 单位？
         * @note does not command the motor immediately
         */
        void Update(bool power_limit_on, float power_limit, float chassis_power,
                    float chassis_power_buffer);

      private:
        // acquired from user
        MotorCANBase** motors_ = nullptr;  // CAN电机数组
        chassis_model_t model_;            // 底盘类型参数

        // pids and current speeds for each motor on the chassis

        ConstrainedPID pids_[MAX_WHEEL_NUM];  // 电机PID控制器数组
        PowerLimit* power_limit_ = nullptr;   // 功耗限制类
        float* speeds_ = nullptr;             // 电机速度数组

        power_limit_t power_limit_info_;
    };

}  // namespace control
