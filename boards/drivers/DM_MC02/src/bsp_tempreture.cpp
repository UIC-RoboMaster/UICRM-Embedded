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

#ifndef NO_ONBOARD_SENSOR
#include "bsp_tempreture.h"

#include "bsp_error_handler.h"

namespace bsp {
    Tempreture::Tempreture(BatteryVol* battery_vol)
        : battery_vol_(battery_vol),
          adc_(&hadc3, ADC_CHANNEL_TEMPSENSOR, ADC_REGULAR_RANK_1, ADC3_SAMPLETIME_640CYCLES_5) {
        RM_ASSERT_TRUE(battery_vol_, "Temperature sensor requires a voltage reference");
        RM_ASSERT_TRUE(hadc3.Init.Resolution == ADC_RESOLUTION_12B, "Temperature ADC must use 12-bit resolution");
    }

    void Tempreture::Start() {
        adc_.Start();
        adc_.Read();  // Discard the first conversion after switching channels.
    }

    void Tempreture::Stop() {
        adc_.Stop();
    }

    uint32_t Tempreture::Read() {
        return adc_.Read();
    }

    float Tempreture::GetTempreture() {
        uint32_t total = 0;
        for (unsigned i = 0; i < 16; ++i)
            total += Read();
        // H723: 12-bit factory calibration at 30/130 C, VREF+ = 3.3 V.
        const int32_t cal1 = *TEMPSENSOR_CAL1_ADDR;
        const int32_t cal2 = *TEMPSENSOR_CAL2_ADDR;
        RM_ASSERT_TRUE(cal1 > 0 && cal2 > cal1 && cal2 < 4095, "Invalid H723 temperature calibration");
        const float normalized = ((total + 8) / 16) * battery_vol_->voltage_vrefint_proportion * 65535.0f / 3.3f;
        return 30.0f + (normalized - cal1) * 100.0f / (cal2 - cal1);
    }
}  // namespace bsp
#endif
