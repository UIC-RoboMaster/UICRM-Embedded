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

#pragma once
#include "bsp_can.h"

namespace control {

    typedef struct {
        float input_voltage;
        float supercap_voltage;
        float input_current;
        float target_power;
    } __packed cap_message_t;

    class SuperCap {
      public:
        SuperCap(bsp::CAN* can, uint16_t rx_id, uint16_t tx_id);
        void UpdateData(const uint8_t data[]);
        void SetTargetPower(float power);
        volatile bool connection_flag_ = false;
        float GetPercent();

        cap_message_t info;

      private:
        bsp::CAN* can_;
        uint16_t supercap_rx_id_;
        uint16_t supercap_tx_id_;
        float percent_;
    };

}  // namespace control