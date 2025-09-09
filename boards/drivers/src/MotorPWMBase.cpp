/*###########################################################
# Copyright (c) 2024. BNU-HKBU UIC RoboMaster              #
#                                                          #
# This program is free software: you can redistribute it   #
# and/or modify it under the terms of the GNU General      #
# Public License as published by the Free Software         #
# Foundation, either version 3 of the License, or (at      #
# your option) any later version.                          #
#                                                          #
# This program is distributed in the hope that it will be  #
# useful, but WITHOUT ANY WARRANTY; without even           #
# the implied warranty of MERCHANTABILITY or FITNESS       #
# FOR A PARTICULAR PURPOSE.  See the GNU General           #
# Public License for more details.                         #
#                                                          #
# You should have received a copy of the GNU General       #
# Public License along with this program.  If not, see     #
# <https://www.gnu.org/licenses/>.                         #
###########################################################*/

#include "MotorPWMBase.h"

#include "utils.h"

namespace driver {
    MotorPWMBase::MotorPWMBase(TIM_HandleTypeDef* htim, uint8_t channel, uint32_t clock_freq,
                               uint32_t output_freq, uint32_t idle_throttle)
        : pwm_(htim, channel, clock_freq, output_freq, idle_throttle),
          idle_throttle_(idle_throttle) {
        pwm_.Start();
    }

    void MotorPWMBase::SetOutput(int16_t val) {
        output_ = val;
        pwm_.SetPulseWidth(val + idle_throttle_);
    }

    void MotorPWMBase::Enable() {
        pwm_.Start();
    }

    void MotorPWMBase::Disable() {
        pwm_.Stop();
    }

    template <typename T>
    T map(T value, T frommin, T frommax, T tomin, T tomax) {
        return ((value - frommin) * (tomax - tomin) / (frommax - frommin) + tomin);
    }

    /*======================== Motor2305 PWM control ========================*/
    Motor2305::Motor2305(TIM_HandleTypeDef* htim, uint8_t channel, uint32_t clock_freq,
                         uint32_t output_freq, uint32_t idle_throttle)
        : MotorPWMBase(htim, channel, clock_freq, output_freq, idle_throttle) {
    }

    void Motor2305::SetOutput(int16_t val) {
        constexpr int16_t MIN_OUTPUT = 0;
        constexpr int16_t MAX_OUTPUT = 700;
        MotorPWMBase::SetOutput(clip<int16_t>(val, MIN_OUTPUT, MAX_OUTPUT));
    }

    /*======================== ServoMG995 PWM control ========================*/
    ServoMG995::ServoMG995(TIM_HandleTypeDef* htim, uint8_t channel, uint32_t clock_freq,
                           uint32_t output_freq, uint32_t idle_throttle)
        : MotorPWMBase(htim, channel, clock_freq, output_freq, idle_throttle) {
    }

    void ServoMG995::SetOutput(int16_t angle) {
        constexpr int16_t MIN_OUTPUT = 500;
        constexpr int16_t MAX_OUTPUT = 2000;
        MotorPWMBase::SetOutput(map<int16_t>(angle, 0, 180, MIN_OUTPUT, MAX_OUTPUT));
    }

    /*======================== Lesar PWM control ========================*/
    Lesar::Lesar(TIM_HandleTypeDef* htim, uint8_t channel, uint32_t clock_freq,
                 uint32_t output_freq, uint32_t idle_throttle)
        : MotorPWMBase(htim, channel, clock_freq, output_freq, idle_throttle) {
    }

    void Lesar::SetOutput(int16_t brightness) {
        constexpr int16_t MIN_OUTPUT = 0;
        constexpr int16_t MAX_OUTPUT = 1000;
        MotorPWMBase::SetOutput(map<int16_t>(brightness, 0, 100, MIN_OUTPUT, MAX_OUTPUT));
    }
}  // namespace driver
