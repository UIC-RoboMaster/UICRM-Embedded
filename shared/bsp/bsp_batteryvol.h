#pragma once
#include "bsp_adc.h"
#include "main.h"
#include "adc.h"
namespace bsp {
    class BatteryVol {
    public:
        BatteryVol(ADC_HandleTypeDef* hadc, uint32_t channel, uint32_t rank, uint32_t sampling_time);
        void InitVREF();
        void Start();
        void Stop();
        uint32_t Read();
        float GetBatteryVol();
        float voltage_vrefint_proportion = 8.0586080586080586080586080586081e-4f;
    private:
        bADC* adc_;
        bADC* adc_vrefint_;
    };
}