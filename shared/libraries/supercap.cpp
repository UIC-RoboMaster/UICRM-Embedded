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

#include "supercap.h"

#include <cstring>

namespace control {

    static void supercap_callback(const uint8_t data[], void* args) {
        SuperCap* supercap = reinterpret_cast<SuperCap*>(args);
        supercap->UpdateData(data);
    }

    SuperCap::SuperCap(bsp::CAN* can, uint16_t rx_id, uint16_t tx_id) {
        can->RegisterRxCallback(rx_id, supercap_callback, this);
        can_ = can;
        supercap_rx_id_ = rx_id;
        supercap_tx_id_ = tx_id;
    }

    void SuperCap::UpdateData(const uint8_t* data) {
        uint16_t data16[4];
        memcpy(data16, data, sizeof(data16));
        info.input_voltage = data16[0] / 100.0f;
        info.supercap_voltage = data16[1] / 100.0f;
        info.input_current = data16[2] / 100.0f;
        info.target_power = data16[3] / 100.0f;
        connection_flag_ = true;
        percent_ = (info.supercap_voltage / info.input_voltage) *
                   (info.supercap_voltage / info.input_voltage);
    }

    float SuperCap::GetPercent() {
        return percent_;
    }

    void SuperCap::SetTargetPower(float power) {
        uint16_t power_int = static_cast<uint16_t>(power * 100);
        uint8_t data[8];
        data[0] = power_int >> 8;
        data[1] = power_int & 0xFF;
        can_->Transmit(supercap_tx_id_, data, 8);
    }

}  // namespace control