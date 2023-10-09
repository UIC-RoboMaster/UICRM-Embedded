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

#include "bsp_adc.h"

#include "string.h"
namespace bsp {
    bADC::bADC(ADC_HandleTypeDef* hadc, uint32_t channel, uint32_t rank, uint32_t sampling_time) {
        hadc_ = hadc;
        channel_ = channel;
        rank_ = rank;
        sampling_time_ = sampling_time;
    }
    void bADC::Start() {
        ADC_ChannelConfTypeDef sConfig;
        memset(&sConfig, 0, sizeof(sConfig));
        sConfig.Channel = channel_;
        sConfig.Rank = rank_;
        sConfig.SamplingTime = sampling_time_;
        if (HAL_ADC_ConfigChannel(hadc_, &sConfig) != HAL_OK) {
            Error_Handler();
        }
    }
    void bADC::Stop() {
        HAL_ADC_Stop(hadc_);
    }
    uint32_t bADC::Read() {
        HAL_ADC_Start(hadc_);
        HAL_ADC_PollForConversion(hadc_, 10);
        return (uint32_t)HAL_ADC_GetValue(hadc_);
    }
}  // namespace bsp