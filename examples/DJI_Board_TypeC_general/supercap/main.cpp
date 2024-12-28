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

#include "bsp_can.h"
#include "bsp_print.h"
#include "cmsis_os.h"
#include "supercap.h"

bsp::CAN* can1 = nullptr;
driver::SuperCap* supercap = nullptr;

void RM_RTOS_Init(void) {
    print_use_uart(&huart1);
    can1 = new bsp::CAN(&hcan1);
    HAL_Delay(1000);
    driver::supercap_init_t supercap_init = {
        .can = can1,
        .tx_id = 0x02e,
        .tx_settings_id = 0x02f,
        .rx_id = 0x030,
    };
    supercap = new driver::SuperCap(supercap_init);
    HAL_Delay(500);
}

void RM_RTOS_Default_Task(const void* arguments) {
    UNUSED(arguments);

    osDelay(2000);
    supercap->SetPowerTotal(120.0f);
    supercap->SetMaxChargePower(150.0f);
    supercap->SetMaxDischargePower(250.0f);
    supercap->SetPerferBuffer(40.0f);
    supercap->Enable();
    supercap->TransmitSettings();
    while (true) {
        set_cursor(0, 0);
        clear_screen();
        supercap->UpdateCurrentBuffer(50.0f);
        print("Cap Voltage: %.2fV\n", supercap->GetCapVoltage());
        print("Cap Power: %.2fW\n", supercap->GetCapPower());
        print("Output Power: %.2fW\n", supercap->GetOutputPower());
        print("Output Voltage: %.2fV\n", supercap->GetOutputVoltage());

        osDelay(50);
    }
}
