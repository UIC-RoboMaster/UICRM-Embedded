#pragma once

#include "controller.h"
#include "motor.h"

namespace control {

    /**
     * @brief Struct for power limit information
     */
    /**
     * @brief 功耗限制变量结构体
     */
    typedef struct {
        float power_limit;                 //  瞬时总功耗限制值
        float WARNING_power;               // 瞬时总功耗警告值
        float WARNING_power_buff;          // 剩余总能量警告值
        float buffer_total_current_limit;  // 剩余总能量限制值
        float power_total_current_limit;
    } power_limit_t;

    /**
     * @brief Class for power limit
     */
    /**
     * @brief 功耗限制类
     */
    class PowerLimit {
      public:
        /**
         * @brief Constructor
         * @param motor_num Number of motors
         */
        /**
         * @brief 构造函数
         * @param motor_num 电机的数量
         */
        PowerLimit(int motor_num);

        /**
         * @brief Limit the output power
         * @param turn_on Whether to turn on the power limit
         * @param power_limit_info Power limit information
         * @param chassis_power Current chassis instant total power, passed in after reading from the referee system
         * @param chassis_power_buffer Current chassis remaining total energy, passed in after reading from the referee system
         * @param PID_output Motor output after PID control *array*, input
         * @param output Motor output after power limit *array*, output
         */
        /**
         * @brief 输出功耗限制
         * @param turn_on 是否开启功耗限制
         * @param power_limit_info 功耗限制信息
         * @param chassis_power 当前底盘瞬时总功耗，从裁判系统读取后传入
         * @param chassis_power_buffer 当前底盘剩余总能量，从裁判系统读取后传入
         * @param PID_output 电机经过PID控制后的输出*数组*，输入
         * @param output 电机经过功耗限制后的输出*数组*，输出
         * @note ?->PID->PowerLimit->?
         */

        void Output(bool turn_on, power_limit_t power_limit_info, float chassis_power,
                    float chassis_power_buffer, float* PID_output, float* output);

      private:
        int motor_num_;
    };

}  // namespace control
