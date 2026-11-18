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
#include "bsp_batteryvol.h"

#include "bsp_error_handler.h"

namespace bsp {
    BatteryVol::BatteryVol(ADC_HandleTypeDef* hadc, uint32_t channel, uint32_t rank, uint32_t sampling_time)
        : adc_(hadc, channel, rank, sampling_time) {
        RM_ASSERT_TRUE(hadc && hadc->Init.Resolution == ADC_RESOLUTION_16B, "Battery ADC must use 16-bit resolution");
    }

    void BatteryVol::InitVREF() {
        RM_ASSERT_TRUE(hadc3.Init.Resolution == ADC_RESOLUTION_12B, "Reference ADC must use 12-bit resolution");
        bADC reference(&hadc3, ADC_CHANNEL_VREFINT, ADC_REGULAR_RANK_1, ADC3_SAMPLETIME_640CYCLES_5);
        reference.Start();
        HAL_Delay(1);      // Allow the internal reference path to settle.
        reference.Read();  // Discard the first conversion after switching channels.
        uint32_t total = 0;
        for (unsigned i = 0; i < 32; ++i)
            total += reference.Read();
        reference.Stop();
        const uint32_t raw = (total + 16) / 32;
        const uint32_t calibration = *VREFINT_CAL_ADDR;
        RM_ASSERT_TRUE(raw > 0 && raw < 4095 && calibration > 0 && calibration < 4095, "Invalid VREFINT data");
        // H723 calibration is 12-bit; the bundled H7 HAL helper assumes 16-bit.
        const float supply = (VREFINT_CAL_VREF / 1000.0f) * calibration / raw;
        RM_ASSERT_TRUE(supply >= 1.62f && supply <= 3.6f, "ADC reference outside H723 operating range");
        voltage_vrefint_proportion = supply / 65535.0f;
    }

    void BatteryVol::Start() {
        adc_.Start();
    }

    void BatteryVol::Stop() {
        adc_.Stop();
    }

    uint32_t BatteryVol::Read() {
        return adc_.Read();
    }

    float BatteryVol::GetBatteryVol() {
        uint32_t total = 0;
        for (unsigned i = 0; i < 16; ++i)
            total += Read();
        // MC02: R86 = 100 kohm, R87 = 10 kohm, divider ratio = 11.
        return ((total + 8) / 16) * voltage_vrefint_proportion * 11.0f;
    }

    float BatteryVol::calcBatteryPercentage(float voltage) {
        float percentage;
        float voltage_2 = voltage * voltage;
        float voltage_3 = voltage_2 * voltage;

        if (voltage < 19.5f) {
            percentage = 0.0f;
        } else if (voltage < 21.9f) {
            percentage = 0.005664f * voltage_3 - 0.3386f * voltage_2 + 6.765f * voltage - 45.17f;
        } else if (voltage < 25.5f) {
            percentage = 0.02269f * voltage_3 - 1.654f * voltage_2 + 40.34f * voltage - 328.4f;
        } else {
            percentage = 1.0f;
        }
        if (percentage < 0.0f) {
            percentage = 0.0f;
        } else if (percentage > 1.0f) {
            percentage = 1.0f;
        }
        return percentage;
    }

    float BatteryVol::GetBatteryPercentage() {
        return calcBatteryPercentage(GetBatteryVol());
    }
}  // namespace bsp
#endif
