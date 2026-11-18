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
 # the implied warranty of MERCHANTABILITY or FITNESS        #
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

namespace bsp {
    /**
     * @brief 板载电池电压采集
     * @note ADC1 CH4 (PC4), 16-bit; InitVREF() shares ADC3 with the temperature sensor.
     *       Call Tempreture::Start() after InitVREF() to restore the temperature channel.
     */
    class BatteryVol {
      public:
        BatteryVol(
            ADC_HandleTypeDef* hadc = &hadc1,
            uint32_t channel = ADC_CHANNEL_4,
            uint32_t rank = ADC_REGULAR_RANK_1,
            uint32_t sampling_time = ADC_SAMPLETIME_64CYCLES_5
        );
        void InitVREF();
        void Start();
        void Stop();
        uint32_t Read();
        float GetBatteryVol();
        static float calcBatteryPercentage(float voltage);
        float GetBatteryPercentage();

        // ADC1 is 16-bit, so one LSB is VDDA / 65535.
        float voltage_vrefint_proportion = 3.3f / 65535.0f;

      private:
        bADC adc_;
    };
}  // namespace bsp
#endif
