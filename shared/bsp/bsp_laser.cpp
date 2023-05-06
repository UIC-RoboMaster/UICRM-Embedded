/*###########################################################
 # Copyright (c) 2023. BNU-HKBU UIC RoboMaster              #
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

#include "bsp_laser.h"

namespace bsp {
    Laser::Laser(TIM_HandleTypeDef* htim, uint32_t channel, uint32_t clock_freq)
        : pwm_(htim, channel, clock_freq, 0, 0) {
        pwm_.Start();
        pwm_.SetFrequency(3921);
    }
    void Laser::SetOutput(uint16_t value) {
        pwm_.SetPulseWidth(value);
    }
} /* namespace bsp */