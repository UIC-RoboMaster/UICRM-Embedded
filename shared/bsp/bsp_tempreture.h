#pragma once
#include "adc.h"
#include "bsp_adc.h"
#include "bsp_batteryvol.h"
#include "main.h"
namespace bsp {
    class Tempreture {
      public:
        Tempreture(bsp::BatteryVol* battery_vol);
        void Start();
        void Stop();
        uint32_t Read();
        float GetTempreture();

      private:
        bADC* adc_;
        float voltage_vrefint_proportion;
    };
}  // namespace bsp