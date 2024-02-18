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

#include "power_limit.h"

namespace control {

    PowerLimit::PowerLimit(int motor_num) {
        motor_num_ = motor_num;
    }

    void PowerLimit::Output(bool turn_on, power_limit_t power_limit_info, float current_power,
                            float current_power_buffer, float* input, float* output) {
        // if not turn on, just output PID output
        // 如果没有开启功率限制，直接输出PID输出
        if (!turn_on) {
            for (int i = 0; i < motor_num_; ++i)
                output[i] = input[i];
            return;
        }

        float total_current_limit;
        // 如果当前可用缓冲能量小于警告缓冲能量
        if (current_power_buffer < power_limit_info.WARNING_power_buff) {
            float power_scale;
            // 如果当前缓冲大于临界值
            if (current_power_buffer > 5.0f) {
                // scale down WARNING_POWER_BUFF
                power_scale = current_power_buffer / power_limit_info.WARNING_power_buff;
            } else {
                // only left 10% of WARNING_POWER_BUFF
                power_scale = 5.0f / power_limit_info.WARNING_power_buff;
            }
            // scale down
            total_current_limit = power_limit_info.buffer_total_current_limit * power_scale;
        } else {
            // power > WARNING_POWER
            if (current_power > power_limit_info.WARNING_power) {
                float power_scale;
                // power < 80w
                if (current_power < power_limit_info.power_limit) {
                    // scale down
                    power_scale = (power_limit_info.power_limit - current_power) /
                                  (power_limit_info.power_limit - power_limit_info.WARNING_power);
                } else {
                    // power > 80w
                    power_scale = 0.0f;
                }
                total_current_limit = power_limit_info.buffer_total_current_limit +
                                      power_limit_info.power_total_current_limit * power_scale;
            } else {
                // power < WARNING_POWER
                total_current_limit = power_limit_info.buffer_total_current_limit +
                                      power_limit_info.power_total_current_limit;
            }
        }
        float total_current = 0;
        for (int i = 0; i < motor_num_; ++i) {
            total_current += fabs(input[i]);
        }
        if (total_current > total_current_limit) {
            float current_scale = total_current_limit / total_current;
            for (int i = 0; i < motor_num_; ++i) {
                output[i] = input[i] * current_scale;
            }
        } else {
            for (int i = 0; i < motor_num_; ++i) {
                output[i] = input[i];
            }
        }
    }

}  // namespace control
