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

#include "can_bridge.h"

#include <string.h>

namespace communication {
    static void can_bridge_callback(const uint8_t data[], const uint32_t ext_id, void* args) {
        CanBridge* bridge = reinterpret_cast<CanBridge*>(args);
        bridge->CallbackWrapper(data, ext_id);
    }
    CanBridge::CanBridge(bsp::CAN* can, uint8_t id) {
        can_ = can;
        id_ = id;
        can_->RegisterRxExtendCallback(id_, can_bridge_callback, this);
    }
    void CanBridge::Send(can_bridge_ext_id_t ext_id, can_bridge_data_t data) {
        ext_id.data.tx_id = id_;
        can_->TransmitExtend(ext_id.id, data.data, 8);
    }

    void CanBridge::CallbackWrapper(const uint8_t* data, const uint32_t ext_id) {
        can_bridge_ext_id_t ext_id_struct;
        ext_id_struct.id = ext_id;
        can_bridge_data_t data_struct;
        memcpy(data_struct.data, data, 8);
        const auto it = reg_to_index_.find(ext_id_struct.data.reg);
        if (it != reg_to_index_.end()) {
            reg_callbacks_[it->second](ext_id_struct, data_struct, reg_args_[it->second]);
        }
    }

    void CanBridge::RegisterRxCallback(uint8_t reg, can_bridge_rx_callback_t callback, void* args) {
        reg_callbacks_[reg_callback_count_] = callback;
        reg_args_[reg_callback_count_] = args;
        reg_to_index_[reg] = reg_callback_count_;
        reg_callback_count_++;
    }
}  // namespace communication
