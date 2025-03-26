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

#ifndef NO_ONBOARD_SENSOR
#include "bsp_batteryvol.h"

namespace bsp {
    BatteryVol::BatteryVol(ADC_HandleTypeDef* hadc, uint32_t channel, uint32_t rank,
                           uint32_t sampling_time) {
        adc_ = new bADC(hadc, channel, rank, sampling_time);
        adc_vrefint_ = new bADC(&hadc1, ADC_CHANNEL_VREFINT, 1, ADC_SAMPLETIME_3CYCLES);

    }
    void BatteryVol::InitVREF() {
        unsigned char i = 0;
        unsigned int total_adc = 0;
        adc_vrefint_->Start();
        for (i = 0; i < 200; i++) {
            total_adc += adc_vrefint_->Read();
        }

        voltage_vrefint_proportion = 200 * 1.2f / total_adc;
    }
    void BatteryVol::Start() {
        adc_->Start();
    }
    void BatteryVol::Stop() {
        adc_->Stop();
    }
    uint32_t BatteryVol::Read() {
        adc_->Start();
        return adc_->Read();
    }

    float BatteryVol::GetBatteryVol() {
#ifdef DJI_BOARD_TYPE_C_GENERAL
        return (float)Read() * voltage_vrefint_proportion * 10.090909090909090909090909090909f;
#endif
#ifdef DJI_BOARD_TYPE_A_GENERAL
        return (float)Read() * voltage_vrefint_proportion * 10.090909090909090909090909090909f;
#endif
#ifdef DM_MC01_GENERAL
        return (float)Read() * voltage_vrefint_proportion * 11.0f;
#endif
        return 0.0f;
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