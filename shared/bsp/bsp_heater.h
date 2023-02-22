#pragma once

#include "bsp_pwm.h"
#include "controller.h"

namespace bsp {

typedef struct {
  TIM_HandleTypeDef *htim;
  uint8_t channel;
  uint32_t clock_freq;
  float temp;
} heater_init_t;

class Heater {
public:
  Heater(heater_init_t init);
  Heater(TIM_HandleTypeDef *htim, uint8_t channel, uint32_t clock_freq,
         float temp);
  float Update(float real_temp);

private:
  PWM pwm_;
  float temp_;
  control::ConstrainedPID pid_;
};

} // namespace bsp