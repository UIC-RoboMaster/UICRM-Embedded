#pragma once

#include "tim.h"

namespace bsp {

class PWM {
public:
  /**
   * @brief constructor for a pwm output manager
   *
   * @param htim         HAL timer handle
   * @param channel      channel associated with the timer, choose from
   * [1,2,3,4]
   * @param clock_freq   clock frequency associated with the timer, in [Hz]
   * @param output_freq  desired pwm output frequency, in [Hz]
   * @param pulse_width  desired pwm output pulse width, is [us]
   */
  PWM(TIM_HandleTypeDef *htim, uint8_t channel, uint32_t clock_freq,
      uint32_t output_freq, uint32_t pulse_width);

  /**
   * @brief start pwm output signal generation
   */
  void Start();

  /**
   * @brief stop pwm output signal generation
   */
  void Stop();

  /**
   * @brief set a new pwm output frequency
   *
   * @param output_freq   desired pwm output frequency, in [Hz]
   */
  void SetFrequency(uint32_t output_freq);

  /**
   * @brief set a new pwm output pulse width
   *
   * @param pulse_width   desired pwm output pulse width, in [us]
   */
  void SetPulseWidth(uint32_t pulse_width);

private:
  TIM_HandleTypeDef *htim_;
  uint32_t channel_;
  uint32_t clock_freq_;
  uint32_t output_freq_;
  uint32_t pulse_width_;
};

} /* namespace bsp */
