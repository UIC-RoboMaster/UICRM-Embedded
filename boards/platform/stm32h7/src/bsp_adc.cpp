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

#include "bsp_adc.h"

#include "bsp_error_handler.h"
namespace bsp {
    bADC::bADC(ADC_HandleTypeDef* hadc, uint32_t channel, uint32_t rank, uint32_t sampling_time) {
        hadc_ = hadc;
        channel_ = channel;
        rank_ = rank;
        sampling_time_ = sampling_time;
    }
    void bADC::Start() {
        // Internal paths (TSEN/VREFEN) can only be enabled with the ADC disabled.
        Stop();
        ADC_ChannelConfTypeDef sConfig = {};
        sConfig.Channel = channel_;
        sConfig.Rank = rank_;
        sConfig.SamplingTime = sampling_time_;
        sConfig.SingleDiff = ADC_SINGLE_ENDED;
        sConfig.OffsetNumber = ADC_OFFSET_NONE;
        if (HAL_ADC_ConfigChannel(hadc_, &sConfig) != HAL_OK) {
            Error_Handler();
        }
    }
    void bADC::Stop() {
        RM_ASSERT_HAL_OK(HAL_ADC_Stop(hadc_), "Failed to stop ADC");
    }
    uint32_t bADC::Read() {
        RM_ASSERT_HAL_OK(HAL_ADC_Start(hadc_), "Failed to start ADC");
        RM_ASSERT_HAL_OK(HAL_ADC_PollForConversion(hadc_, 10), "ADC conversion timed out");
        return (uint32_t)HAL_ADC_GetValue(hadc_);
    }
}  // namespace bsp
