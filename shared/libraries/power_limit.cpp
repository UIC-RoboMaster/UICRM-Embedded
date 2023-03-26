#include "power_limit.h"

namespace control {

    PowerLimit::PowerLimit(int motor_num) {
        motor_num_ = motor_num;
    }

    void PowerLimit::Output(bool turn_on, power_limit_t power_limit_info, float chassis_power,
                            float chassis_power_buffer, float* PID_output, float* output) {
        // 不使用功耗限制，直接原样输出
        if (!turn_on) {
            for (int i = 0; i < motor_num_; ++i)
                output[i] = PID_output[i];
            return;
        }

        // 输入电机的总电流限制值
        float total_current_limit;
        if (chassis_power_buffer < power_limit_info.WARNING_power_buff) {
            float power_scale;
            if (chassis_power_buffer > 5.0f) {
                // scale down WARNING_POWER_BUFF
                power_scale = chassis_power_buffer / power_limit_info.WARNING_power_buff;
            } else {
                // only left 10% of WARNING_POWER_BUFF
                power_scale = 5.0f / power_limit_info.WARNING_power_buff;
            }
            // scale down
            total_current_limit = power_limit_info.buffer_total_current_limit * power_scale;
        } else {
            if (chassis_power > power_limit_info.WARNING_power) {
                // power > WARNING_POWER
                float power_scale;
                if (chassis_power < power_limit_info.power_limit) {
                    // power < 80w, scale down
                    power_scale = (power_limit_info.power_limit - chassis_power) /
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
        // 计算输入电机总电流
        float total_current = 0;
        for (int i = 0; i < motor_num_; ++i)
            total_current += fabs(PID_output[i]);
        if (total_current > total_current_limit) {
            // 输入电机总电流>限制值，需要scale down
            float current_scale = total_current_limit / total_current;
            for (int i = 0; i < motor_num_; ++i)
                output[i] = PID_output[i] * current_scale;
        } else {
            // 输入电机总电流<限制值，原样输出
            for (int i = 0; i < motor_num_; ++i)
                output[i] = PID_output[i];
        }
    }

}  // namespace control
