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

#include "Motor2006.h"

#include "utils.h"

namespace driver {

Motor2006::Motor2006(bsp::CAN* can, uint16_t rx_id, uint16_t tx_id)
    : DjiMotorBase(can, rx_id, tx_id) {
    // M2006 RX_ID = 0x200 + 电调 ID
    // TX_ID 自动识别逻辑见 ResolveTxId()
    if (tx_id == 0x00) {
        RM_ASSERT_GE(rx_id, 0x201, "Invalid rx id for M2006");
        state_.tx_id = ResolveTxId(rx_id);
    }
    state_.transmission_ratio = Motor2006Config::ORIGINAL_TRANSMISSION_RATIO;
    torque_constant_ = Motor2006Config::RATED_TORQUE_CONSTANT;
    CanMotorBase::RegisterCanCallback(can, rx_id, &Motor2006::RxThunk, this);
}

void Motor2006::RxThunk(void* ctx, const uint8_t data[]) {
    static_cast<Motor2006*>(ctx)->UpdateData(data);
}

void Motor2006::UpdateData(const uint8_t data[]) {
    state_.raw_theta = data[0] << 8 | data[1];
    state_.raw_omega = data[2] << 8 | data[3];
    state_.raw_current = data[4] << 8 | data[5];

    constexpr float OMEGA_SCALE = 2 * PI / 60;
    state_.theta = uint_to_float(state_.raw_theta, 0, 2 * PI, Motor2006Config::ENCODER_BITS);
    state_.omega = state_.raw_omega * OMEGA_SCALE;

    FinishFeedbackUpdate();
}

void Motor2006::PrintData() const {
    print("online: %s ", (IsOnline() ? "true" : "false"));
    print("theta: % .4f ", GetTheta());
    print("output shaft theta: % .4f ", GetOutputShaftTheta());
    print("omega: % .4f ", GetOmega());
    print("output shaft omega: % .4f ", GetOutputShaftOmega());
    print("raw current get: % d \r\n", state_.raw_current);
}

void Motor2006::SetOutput(int16_t val) {
    output_ = clip<int16_t>(val, -Motor2006Config::MAX_OUTPUT_CURRENT,
                            Motor2006Config::MAX_OUTPUT_CURRENT);
}

}  // namespace driver
