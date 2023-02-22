#include "bsp_tempreture.h"
namespace bsp {
Tempreture::Tempreture(bsp::BatteryVol *battery_vol) {
  voltage_vrefint_proportion = battery_vol->voltage_vrefint_proportion;
  adc_ = new bADC(&hadc1, ADC_CHANNEL_TEMPSENSOR, 1, ADC_SAMPLETIME_3CYCLES);
}
void Tempreture::Start() { adc_->Start(); }
void Tempreture::Stop() { adc_->Stop(); }
uint32_t Tempreture::Read() { return adc_->Read(); }
float Tempreture::GetTempreture() {
  return ((float)Read() * voltage_vrefint_proportion - 0.76f) * 400.0f + 25.0f;
}
} // namespace bsp