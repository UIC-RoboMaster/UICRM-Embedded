/*###########################################################
 # Copyright (c) 2023-2024. BNU-HKBU UIC RoboMaster         #
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

#pragma once
#include "main.h"

namespace bsp {

    class bADC {
      public:
        bADC(ADC_HandleTypeDef* hadc, uint32_t channel, uint32_t rank, uint32_t sampling_time);
        void Start();
        void Stop();
        uint32_t Read();

      private:
        ADC_HandleTypeDef* hadc_;
        uint32_t channel_;
        uint32_t rank_;
        uint32_t sampling_time_;
    };
}  // namespace bsp