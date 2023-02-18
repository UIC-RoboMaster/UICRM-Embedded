#pragma once
#include "main.h"
#include "bsp_pwm.h"
namespace bsp {
    class Laser {
    public:
        Laser(TIM_HandleTypeDef* htim, uint32_t channel, uint32_t clock_freq);
        void SetVal(uint16_t value);
    private:
        PWM pwm_;
    };
}