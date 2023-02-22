#include "bsp_laser.h"

namespace bsp {
Laser::Laser(TIM_HandleTypeDef *htim, uint32_t channel, uint32_t clock_freq)
    : pwm_(htim, channel, clock_freq, 0, 0) {
  pwm_.Start();
  pwm_.SetFrequency(3921);
}
void Laser::SetVal(uint16_t value) { pwm_.SetPulseWidth(value); }
} /* namespace bsp */