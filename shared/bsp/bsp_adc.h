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
}