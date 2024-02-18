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

#include "main.h"

#include "bsp_batteryvol.h"
#include "bsp_gpio.h"
#include "bsp_print.h"
#include "bsp_tempreture.h"
#include "cmsis_os.h"
bsp::BatteryVol* battery_vol;
bsp::Tempreture* tempreture;

void RM_RTOS_Init() {
    print_use_uart(&huart4);
    battery_vol = new bsp::BatteryVol(&hadc1, ADC_CHANNEL_0, 3, ADC_SAMPLETIME_3CYCLES);
    tempreture = new bsp::Tempreture(battery_vol);
    battery_vol->Start();
    tempreture->Start();
    HAL_Delay(500);
}

void RM_RTOS_Default_Task(const void* args) {
    UNUSED(args);
    float voltage = 0;
    float now_tempreture = 0;
    int vol_value = 0;
    int tem_value = 0;
    while (true) {
        set_cursor(0, 0);
        clear_screen();
        vol_value = battery_vol->Read();
        tem_value = tempreture->Read();
        voltage = battery_vol->GetBatteryVol();
        now_tempreture = tempreture->GetTempreture();
        print("Battery Vol Value: %d\n", vol_value);
        print("Tempreture Value: %d\n", tem_value);
        print("Battery Voltage: %f\n", voltage);
        print("Tempreture: %f\n", now_tempreture);
        osDelay(100);
    }
}
