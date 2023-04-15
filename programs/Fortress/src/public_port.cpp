#include "public_port.h"
bsp::CAN* can1 = nullptr;
bsp::CAN* can2 = nullptr;
bsp::BatteryVol* battery_vol = nullptr;
void init_can() {
    can1 = new bsp::CAN(&hcan1, 0x201, true);
    can2 = new bsp::CAN(&hcan2, 0x205, false);
}
void init_batt() {
    battery_vol = new bsp::BatteryVol(&hadc3, ADC_CHANNEL_8, 1, ADC_SAMPLETIME_3CYCLES);
}