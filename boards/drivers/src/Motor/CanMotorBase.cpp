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

#include "CanMotorBase.h"

using namespace bsp;

namespace driver {

CanMotorBase::CanMotorBase(uint32_t online_threshold) : ConnectionDriver(online_threshold) {}

void CanMotorBase::RegisterCanCallback(CAN* can, uint16_t rx_id, CanRxHandler handler, void* ctx) {
    rx_handler_ = handler;
    rx_ctx_ = ctx;
    can->RegisterRxCallback(rx_id, &CanMotorBase::BspRxThunk, this);
}

void CanMotorBase::BspRxThunk(const uint8_t data[], void* args) {
    auto* self = static_cast<CanMotorBase*>(args);
    RM_ASSERT_TRUE(self->rx_handler_ != nullptr, "CAN RX handler not set");
    self->rx_handler_(self->rx_ctx_, data);
}

void CanMotorBase::TransmitFrame(bsp::CAN* can, uint16_t tx_id, const uint8_t data[8], uint8_t dlc) {
    can->Transmit(tx_id, data, dlc);
}

}  // namespace driver
