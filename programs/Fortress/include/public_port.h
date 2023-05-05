#pragma once
#include "bsp_batteryvol.h"
#include "bsp_can.h"
extern bsp::CAN* can1;
extern bsp::CAN* can2;
extern bsp::BatteryVol* battery_vol;
void init_can();
void init_batt();
