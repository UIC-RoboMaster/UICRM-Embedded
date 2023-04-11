#pragma once
#include "bsp_pwm.h"
#include "main.h"
namespace bsp {
    class Laser {
      public:
        Laser(TIM_HandleTypeDef* htim, uint32_t channel, uint32_t clock_freq);
        void SetOutput(uint16_t value);

      private:
        PWM pwm_;
    };
}  // namespace bsp