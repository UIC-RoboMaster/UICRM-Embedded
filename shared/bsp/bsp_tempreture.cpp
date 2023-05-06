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

#include "bsp_tempreture.h"
namespace bsp {
    Tempreture::Tempreture(bsp::BatteryVol* battery_vol) {
        voltage_vrefint_proportion = battery_vol->voltage_vrefint_proportion;
        adc_ = new bADC(&hadc1, ADC_CHANNEL_TEMPSENSOR, 1, ADC_SAMPLETIME_3CYCLES);
    }
    void Tempreture::Start() {
        adc_->Start();
    }
    void Tempreture::Stop() {
        adc_->Stop();
    }
    uint32_t Tempreture::Read() {
        return adc_->Read();
    }
    float Tempreture::GetTempreture() {
        return ((float)Read() * voltage_vrefint_proportion - 0.76f) * 400.0f + 25.0f;
    }
}  // namespace bsp