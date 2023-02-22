#pragma once

#include "bsp_pwm.h"

namespace display {

const uint32_t color_red = 0xFFFF0000;
const uint32_t color_green = 0xFF00FF00;
const uint32_t color_blue = 0xFF0000FF;
const uint32_t color_yellow = 0xFFFFFF00;
const uint32_t color_cyan = 0xFF00FFFF;
const uint32_t color_magenta = 0xFFFF00FF;

class RGB {
public:
  RGB(TIM_HandleTypeDef *htim, uint8_t channelR, uint8_t channelG,
      uint8_t channelB, uint32_t clock_freq);
  void Display(uint32_t aRGB);
  void Stop();

private:
  bsp::PWM R_, G_, B_;
};

} // namespace display
