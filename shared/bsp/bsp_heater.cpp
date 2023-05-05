#include "bsp_heater.h"

namespace bsp {

    Heater::Heater(TIM_HandleTypeDef* htim, uint8_t channel, uint32_t clock_freq, float temp)
        : pwm_(htim, channel, clock_freq, 2000, 0), pid_() {
        temp_ = temp;
        pwm_.Start();
        float* pid_param = new float[3]{160, 0.1, 0};
        float heater_I_limit = 800;
        float heater_output_limit = 500;
        pid_.Reinit(pid_param, heater_I_limit, heater_output_limit);
    }

    Heater::Heater(heater_init_t init)
        : pwm_(init.htim, init.channel, init.clock_freq, 2000, 0), pid_() {
        temp_ = init.temp;
        pwm_.Start();
        float* pid_param = new float[3]{160, 0.1, 0};
        float heater_I_limit = 800;
        float heater_output_limit = 500;
        pid_.Reinit(pid_param, heater_I_limit, heater_output_limit);
    }

    float Heater::Update(float real_temp) {
        if (real_temp < temp_ - 1)
            pid_.cumulated_err_ = 0;
        float output = pid_.ComputeOutput(temp_ - real_temp);
        output = output > 0 ? output : 0;
        if (real_temp > temp_ + 0.5)
            output = 0;
        pwm_.SetPulseWidth((uint32_t)output);
        return output;
    }
}  // namespace bsp