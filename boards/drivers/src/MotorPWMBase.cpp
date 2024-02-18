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

    void Motor2305::SetOutput(int16_t val) {
        constexpr int16_t MIN_OUTPUT = 0;
        constexpr int16_t MAX_OUTPUT = 700;
        MotorPWMBase::SetOutput(clip<int16_t>(val, MIN_OUTPUT, MAX_OUTPUT));
    }
}
