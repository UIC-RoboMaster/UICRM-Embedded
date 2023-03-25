#pragma once

#include "controller.h"
#include "motor.h"

namespace control {

    typedef struct {
        float power_limit;
        float WARNING_power;
        float WARNING_power_buff;
        float buffer_total_current_limit;
        float power_total_current_limit;
    } power_limit_t;

    class PowerLimit {
      public:
        PowerLimit(int motor_num);
        void Output(bool turn_on, power_limit_t power_limit_info, float chassis_power,
                    float chassis_power_buffer, float* PID_output, float* output);

      private:
        int motor_num_;
    };

}  // namespace control
