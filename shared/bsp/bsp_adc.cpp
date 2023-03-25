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