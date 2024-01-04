/*###########################################################
 # Copyright (c) 2024. BNU-HKBU UIC RoboMaster              #
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

//
// Created by yangr on 2024/1/4.
//

#include "supercap.h"
namespace driver {
    SuperCap::SuperCap(driver::supercap_init_t init) {
        can_ = init.can;
        tx_id_ = init.tx_id;
        tx_settings_id_ = init.tx_settings_id;
        rx_id_ = init.rx_id;

        can_->RegisterRxCallback(rx_id_, CallbackWrapper, this);
    }
    void SuperCap::CallbackWrapper(const uint8_t* data, void* args) {
        SuperCap* supercap = reinterpret_cast<SuperCap*>(args);
        supercap->UpdateData(data);
    }
    void SuperCap::UpdateData(const uint8_t* data) {
        int16_t cap_voltage = (int16_t)data[0] << 8 | (int16_t)data[1];
        cap_voltage_ = 0.01f * cap_voltage;
        int16_t cap_power = (int16_t)data[2] << 8 | (int16_t)data[3];
        cap_power_ = 0.01f * cap_power;
        int16_t output_power = (int16_t)data[4] << 8 | (int16_t)data[5];
        output_power_ = 0.01f * output_power;
        status_.data = data[6];
        output_voltage_ = 0.1f * data[7];

        connection_flag_ = true;
    }
    void SuperCap::Enable() {
        tx_flags_.flags.enable = true;
    }

    void SuperCap::Disable() {
        tx_flags_.flags.enable = false;
    }
    void SuperCap::SetPowerTotal(float power) {
        power_total_ = power;
    }
    void SuperCap::SetMaxChargePower(float power) {
        max_charge_power_ = power;
    }
    void SuperCap::SetMaxDischargePower(float power) {
        max_discharge_power_ = power;
    }
    void SuperCap::SetPerferBuffer(float buffer) {
        perfer_buffer_ = buffer;
    }
    void SuperCap::TransmitSettings() {
        uint8_t data[8];
        data[0] = (uint8_t)power_total_;
        data[2] = (uint8_t)max_discharge_power_;
        data[4] = (uint8_t)max_charge_power_;
        data[6] = tx_flags_.data;
        data[7] = (uint8_t)perfer_buffer_;
        can_->Transmit(tx_settings_id_, data, 8);
    }
    void SuperCap::UpdateCurrentBuffer(float buffer) {
        uint8_t data[8] = {0, 0, 0, 0, 0, 0, 0, 0};
        data[0] = (uint8_t)((uint16_t)(buffer * 100) >> 8);
        data[1] = (uint8_t)((uint16_t)(buffer * 100) & 0xff);
        can_->Transmit(tx_id_, data, 8);
    }
    float SuperCap::GetCapVoltage() const {
        return cap_voltage_;
    }
    float SuperCap::GetCapPower() const {
        return cap_power_;
    }
    float SuperCap::GetOutputPower() const {
        return output_power_;
    }
    float SuperCap::GetOutputVoltage() const {
        return output_voltage_;
    }
    supercap_status_t SuperCap::GetStatus() const {
        return status_;
    }
}  // namespace driver
