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

#include "referee_task.h"

osThreadId_t refereeTaskHandle;

osThreadId_t refereercTaskHandle;

void RefereeUART::RxCompleteCallback() {
    osThreadFlagsSet(*refereeTaskHandle_, RX_SIGNAL);
}

RefereeUART* referee_uart = nullptr;
RefereeUART* refereerc_uart = nullptr;
communication::Referee* referee = nullptr;
communication::Referee* refereerc = nullptr;

void refereeTask(void* arg) {
    UNUSED(arg);
    uint32_t length;
    uint8_t* data;
    osDelay(500);
    while (true) {
        /* wait until rx data is available */
        uint32_t flags = osThreadFlagsWait(RX_SIGNAL, osFlagsWaitAll, osWaitForever);
        if (flags & RX_SIGNAL) {  // unnecessary check
            /* time the non-blocking rx / tx calls (should be <= 1 osTick) */
            length = referee_uart->Read(&data);
            referee->Receive(communication::package_t{data, (int)length});
        }
    }
}

void refereercTask(void* arg) {
    UNUSED(arg);
    uint32_t length;
    uint8_t* data;
    osDelay(500);
    while (true) {
        /* wait until rx data is available */
        uint32_t flags = osThreadFlagsWait(RX_SIGNAL, osFlagsWaitAll, osWaitForever);
        if (flags & RX_SIGNAL) {  // unnecessary check
            /* time the non-blocking rx / tx calls (should be <= 1 osTick) */
            length = refereerc_uart->Read(&data);
            refereerc->Receive(communication::package_t{data, (int)length});
        }
    }
}

void init_referee() {
    referee_uart = new RefereeUART(&huart3, &refereeTaskHandle);
    referee_uart->SetupRx(300);
    referee_uart->SetupTx(300);
    referee = new communication::Referee;

//    refereerc_uart = new RefereeUART(&huart1, &refereercTaskHandle);
//    refereerc_uart->SetupRx(300);
//    refereerc_uart->SetupTx(300);
//    refereerc = new communication::Referee;
}
