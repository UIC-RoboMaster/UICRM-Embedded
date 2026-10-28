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

#include "referee_task.h"

bsp::UART* referee_uart = nullptr;
communication::Referee* referee = nullptr;
controller::RefereeRC* refereerc = nullptr;

void init_referee() {
    referee_uart = new bsp::UART(&huart6);
    referee_uart->SetupRx(300);
    referee_uart->SetupTx(300);
    referee = new communication::Referee(referee_uart);

    refereerc = new controller::RefereeRC(&huart1);
}
