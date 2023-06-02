/*###########################################################
 # Copyright (c) 2023. BNU-HKBU UIC RoboMaster              #
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

#include <cstring>

#include "bsp_gpio.h"
#include "bsp_uart.h"
#include "cmsis_os.h"
#include "main.h"
#include "protocol.h"
#include "rgb.h"

static communication::Host* host = nullptr;
static bsp::UART* client = nullptr;
static bsp::GPIO* led = nullptr;

void RM_RTOS_Init(void) {
    client = new bsp::UART(&huart1);
    client->SetupTx(300);
    client->SetupRx(300);

    host = new communication::Host;

    led = new bsp::GPIO(LED_GPIO_Port, LED_Pin);
    led->High();
}

void RM_RTOS_Default_Task(const void* argument) {
    UNUSED(argument);
    communication::package_t frame;
    communication::pack_t message[3] = {
        {"Sazabi Gundam!"}, {"Sinanju Gundam!"}, {"Kshatriya Gundam!"}};

    while (true) {
        for (int i = 0; i < 3; ++i) {
            memcpy(&host->pack, message + i, sizeof(communication::pack_t));
            frame = host->Transmit(communication::PACK);
            client->Write(frame.data, frame.length);
            led->Low();
            osDelay(200);
            led->High();
            osDelay(300);
        }
    }
}