#include "bsp_batteryvol.h"
namespace bsp {
BatteryVol::BatteryVol(ADC_HandleTypeDef *hadc, uint32_t channel, uint32_t rank,
                       uint32_t sampling_time) {
  adc_ = new bADC(hadc, channel, rank, sampling_time);
  adc_vrefint_ =
      new bADC(&hadc1, ADC_CHANNEL_VREFINT, 1, ADC_SAMPLETIME_3CYCLES);
  adc_vrefint_->Start();
}
void BatteryVol::InitVREF() {
  unsigned char i = 0;
  unsigned int total_adc = 0;
  for (i = 0; i < 200; i++) {
    total_adc += adc_vrefint_->Read();
  }

  voltage_vrefint_proportion = 200 * 1.2f / total_adc;
}
void BatteryVol::Start() { adc_->Start(); }
void BatteryVol::Stop() { adc_->Stop(); }
uint32_t BatteryVol::Read() { return adc_->Read(); }
float BatteryVol::GetBatteryVol() {
  return (float)Read() * voltage_vrefint_proportion *
         10.090909090909090909090909090909f;
}
} // namespace bsp