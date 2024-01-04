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

#pragma once

#include "bsp_can.h"

namespace driver {
    typedef struct {
        bsp::CAN* can;
        uint16_t tx_id;
        uint16_t tx_settings_id;
        uint16_t rx_id;
    } supercap_init_t;

    typedef union {
        uint8_t data;
        struct {
            uint8_t enable : 1;
            uint8_t cap_low_voltage : 1;
            uint8_t power_overload : 1;
            uint8_t : 3;
            uint8_t can_error : 1;
            uint8_t cap_error : 1;
        } __packed flags;
    } supercap_status_t;

    typedef union {
        uint8_t data;
        struct {
            uint8_t enable : 1;
            uint8_t : 7;
        } __packed flags;
    } supercap_send_flags_t;

    class SuperCap {
      public:
        SuperCap(supercap_init_t init);

        void Enable();

        void Disable();

        void SetPowerTotal(float power);

        void SetMaxChargePower(float power);

        void SetMaxDischargePower(float power);

        void SetPerferBuffer(float buffer);

        void TransmitSettings();

        void UpdateCurrentBuffer(float buffer);

        float GetCapVoltage() const;

        float GetCapPower() const;

        float GetOutputPower() const;

        float GetOutputVoltage() const;

        supercap_status_t GetStatus() const;

        void UpdateData(const uint8_t* data);

        static void CallbackWrapper(const uint8_t* data, void* args);

      private:
        bsp::CAN* can_;
        uint16_t tx_id_;
        uint16_t tx_settings_id_;
        uint16_t rx_id_;

        float cap_voltage_ = 0.0f;
        float cap_power_ = 0.0f;
        float output_power_ = 0.0f;
        float output_voltage_ = 0.0f;
        supercap_status_t status_ = {0};

        float power_total_ = 0.0f;
        float max_charge_power_ = 150.0f;
        float max_discharge_power_ = 250.0f;
        float perfer_buffer_ = 45.0f;
        supercap_send_flags_t tx_flags_ = {0};

        volatile bool connection_flag_ = false;
    };
}  // namespace driver
