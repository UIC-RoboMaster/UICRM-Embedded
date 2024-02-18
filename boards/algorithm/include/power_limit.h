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

#include <math.h>

#include "main.h"

namespace control {

    /**
     * @brief 电机限流信息
     */
    /**
     * @brief Motor current limit information
     */
    typedef struct {
        float power_limit;                 // 电机功率限制，单位为W
        float WARNING_power;               // 电机功率警告，单位为W
        float WARNING_power_buff;          // 电机缓冲能量警告，单位为J
        float buffer_total_current_limit;  // 电机缓冲总电流限制，单位为A
        float power_total_current_limit;   // 电机功率总电流限制，单位为A
    } power_limit_t;

    /**
     * @brief 电机功率限制
     * @details 用于限制电机功率，防止电机过载
     *
     */
    /**
     * @brief Motor power limit
     * @details Used to limit motor power to prevent motor overload
     */
    class PowerLimit {
      public:
        /**
         * @brief 构造函数
         * @param motor_num 电机的数量
         */
        PowerLimit(int motor_num);
        /**
         * @brief 电机功率限制输出
         * @param turn_on 是否开启功率限制，如果不开启则直接输出PID输出
         * @param power_limit_info 电源功率限制信息
         * @param current_power 电源当前功率，单位为W
         * @param current_power_buffer 电源当前可用缓冲能量，单位为J
         * @param input 需要被限制的原始输出，通常为PID的输出
         * @param output 被限制后的输出
         */
        /**
         * @brief Motor power limit output
         * @param turn_on Whether to enable power limit, if not, output PID output directly
         * @param power_limit_info Power limit information
         * @param current_power Current power of the power supply, in W
         * @param current_power_buffer Current available buffer energy of the power supply, in J
         * @param input Original output to be limited, usually the output of PID
         * @param output Limited output
         */
        void Output(bool turn_on, power_limit_t power_limit_info, float current_power,
                    float current_power_buffer, float* input, float* output);

      private:
        int motor_num_;
    };

}  // namespace control
