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

#include "Motor3508.h"

#include "utils.h"

namespace driver {

Motor3508::Motor3508(bsp::CAN* can, uint16_t rx_id, uint16_t tx_id)
    : DjiMotorBase(can, rx_id, tx_id) {
    // M3508 RX_ID = 0x200 + 电机 ID
    // TX_ID 自动识别逻辑见 ResolveTxId()
    if (tx_id == 0x00) {
        RM_ASSERT_GE(rx_id, 0x201, "Invalid rx id for M3508");
        state_.tx_id = ResolveTxId(rx_id);
    }
    state_.transmission_ratio = Motor3508Config::ORIGINAL_TRANSMISSION_RATIO;
    torque_constant_ = Motor3508Config::RATED_TORQUE_CONSTANT;
    CanMotorBase::RegisterCanCallback(can, rx_id, &Motor3508::RxThunk, this);
}

void Motor3508::RxThunk(void* ctx, const uint8_t data[]) {
    static_cast<Motor3508*>(ctx)->UpdateData(data);
}

void Motor3508::UpdateData(const uint8_t data[]) {
    state_.raw_theta = data[0] << 8 | data[1];
    state_.raw_omega = data[2] << 8 | data[3];
    state_.raw_current = data[4] << 8 | data[5];
    state_.raw_temperature = data[6];

    constexpr float OMEGA_SCALE = 2 * PI / 60;
    state_.theta = uint_to_float(state_.raw_theta, 0, 2 * PI, Motor3508Config::ENCODER_BITS);
    state_.omega = state_.raw_omega * OMEGA_SCALE;

    FinishFeedbackUpdate();
}

void Motor3508::PrintData() const {
    print("online: %s ", (IsOnline() ? "true" : "false"));
    print("theta: % .4f ", GetTheta());
    print("output shaft theta: % .4f ", GetOutputShaftTheta());
    print("omega: % .4f ", GetOmega());
    print("output shaft omega: % .4f ", GetOutputShaftOmega());
    print("raw temperature: %3d ", state_.raw_temperature);
    print("raw current get: % d \r\n", state_.raw_current);
}

void Motor3508::SetOutput(int16_t val) {
    output_ = clip<int16_t>(val, -Motor3508Config::MAX_OUTPUT_CURRENT,
                            Motor3508Config::MAX_OUTPUT_CURRENT);
}

}  // namespace driver
