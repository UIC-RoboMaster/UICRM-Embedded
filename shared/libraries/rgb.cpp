#include "rgb.h"

namespace display {

RGB::RGB(TIM_HandleTypeDef* htim, uint8_t channelR, uint8_t channelG, uint8_t channelB,
         uint32_t clock_freq)
    : R_(htim, channelR, clock_freq, 0, 0),
      G_(htim, channelG, clock_freq, 0, 0),
      B_(htim, channelB, clock_freq, 0, 0) {
  R_.Start();
  G_.Start();
  B_.Start();
  R_.SetFrequency(3921);
  G_.SetFrequency(3921);
  B_.SetFrequency(3921);
}

void RGB::Display(uint32_t aRGB) {
  volatile uint8_t alpha;
  volatile uint16_t red, green, blue;

  alpha = (aRGB & 0xFF000000) >> 24;
  red = ((aRGB & 0x00FF0000) >> 16) * alpha / 255.0;
  green = ((aRGB & 0x0000FF00) >> 8) * alpha / 255.0;
  blue = ((aRGB & 0x000000FF) >> 0) * alpha / 255.0;

  R_.SetPulseWidth(red);
  G_.SetPulseWidth(green);
  B_.SetPulseWidth(blue);
}

void RGB::Stop() {
  R_.Stop();
  G_.Stop();
  B_.Stop();
}

}  // namespace display
