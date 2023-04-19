#include "bsp_batteryvol.h"
namespace bsp {
    BatteryVol::BatteryVol(ADC_HandleTypeDef* hadc, uint32_t channel, uint32_t rank,
                           uint32_t sampling_time) {
        adc_ = new bADC(hadc, channel, rank, sampling_time);
        adc_vrefint_ = new bADC(&hadc1, ADC_CHANNEL_VREFINT, 1, ADC_SAMPLETIME_3CYCLES);
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
    void BatteryVol::Start() {
        adc_->Start();
    }
    void BatteryVol::Stop() {
        adc_->Stop();
    }
    uint32_t BatteryVol::Read() {
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