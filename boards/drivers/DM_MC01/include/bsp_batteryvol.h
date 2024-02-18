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
#ifndef NO_ONBOARD_SENSOR
#include "adc.h"
#include "bsp_adc.h"
#include "main.h"

namespace bsp {
    class BatteryVol {
      public:
        BatteryVol(ADC_HandleTypeDef* hadc, uint32_t channel, uint32_t rank,
                   uint32_t sampling_time);
        void InitVREF();
        void Start();
        void Stop();
        uint32_t Read();
        float GetBatteryVol();
        static float calcBatteryPercentage(float voltage);
        float GetBatteryPercentage();
        float voltage_vrefint_proportion = 8.0586080586080586080586080586081e-4f;

      private:
        bADC* adc_;
        bADC* adc_vrefint_;
    };
}  // namespace bsp
#endif