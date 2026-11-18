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
 # the implied warranty of MERCHANTABILITY or FITNESS        #
 # FOR A PARTICULAR PURPOSE.  See the GNU General           #
 # Public License for more details.                         #
 #                                                          #
 # You should have received a copy of the GNU General       #
 # Public License along with this program.  If not, see     #
 # <https://www.gnu.org/licenses/>.                         #
 ###########################################################*/

#include "main.h"

#include "bsp_batteryvol.h"
#include "bsp_print.h"
#include "bsp_tempreture.h"
#include "cmsis_os.h"

static bsp::BatteryVol* battery_vol = nullptr;
static bsp::Tempreture* tempreture = nullptr;

void RM_RTOS_Init() {
    print_use_rtt();
    battery_vol = new bsp::BatteryVol();
    battery_vol->InitVREF();
    battery_vol->Start();
    tempreture = new bsp::Tempreture(battery_vol);
    tempreture->Start();
}

void RM_RTOS_Default_Task(const void* args) {
    UNUSED(args);
    while (true) {
        // ADC3 is shared: measure VREFINT, then restore the temperature channel.
        battery_vol->InitVREF();
        tempreture->Start();
        const float voltage = battery_vol->GetBatteryVol();
        const float temperature = tempreture->GetTempreture();

        set_cursor(0, 0);
        clear_screen();
        print("Battery Voltage: %.3f V\r\n", voltage);
        print("Battery Percentage: %.1f %%\r\n", bsp::BatteryVol::calcBatteryPercentage(voltage) * 100);
        print("MCU Temperature: %.2f C\r\n", temperature);
        osDelay(250);
    }
}
