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

#include "Motor6020.h"

#include "utils.h"

namespace driver {

Motor6020::Motor6020(bsp::CAN* can, uint16_t rx_id, uint16_t tx_id)
    : DjiMotorBase(can, rx_id, tx_id) {
    if (tx_id == 0x00) {
        RM_ASSERT_GE(rx_id, 0x205, "Invalid rx id for GM6020");
        state_.tx_id = ResolveTxId(rx_id);
    }
    state_.transmission_ratio = Motor6020Config::ORIGINAL_TRANSMISSION_RATIO;
    torque_constant_ = Motor6020Config::RATED_TORQUE_CONSTANT;
    // 绝对位置电机不需要初始化 align_angle_
    state_.power_on_angle = 0;
    CanMotorBase::RegisterCanCallback(can, rx_id, &Motor6020::RxThunk, this);
}

void Motor6020::RxThunk(void* ctx, const uint8_t data[]) {
    static_cast<Motor6020*>(ctx)->UpdateData(data);
}

void Motor6020::UpdateData(const uint8_t data[]) {
    state_.raw_theta = data[0] << 8 | data[1];
    state_.raw_omega = data[2] << 8 | data[3];
    state_.raw_current = data[4] << 8 | data[5];
    state_.raw_temperature = data[6];

    constexpr float OMEGA_SCALE = 2 * PI / 60;
    state_.theta = uint_to_float(state_.raw_theta, 0, 2 * PI, Motor6020Config::ENCODER_BITS);
    state_.omega = (state_.raw_omega * OMEGA_SCALE) * input_speed_filter_
                 + state_.omega * (1 - input_speed_filter_);

    FinishFeedbackUpdate();
}

void Motor6020::PrintData() const {
    print("online: %s ", (IsOnline() ? "true" : "false"));
    print("theta: % .4f ", GetTheta());
    print("output shaft theta: % .4f ", GetOutputShaftTheta());
    print("omega: % .4f ", GetOmega());
    print("output shaft omega: % .4f ", GetOutputShaftOmega());
    print("raw temperature: %3d ", state_.raw_temperature);
    print("raw current get: % d \r\n", state_.raw_current);
}

void Motor6020::SetOutput(int16_t val) {
    output_ = clip<int16_t>(val, -Motor6020Config::MAX_OUTPUT_CURRENT,
                            Motor6020Config::MAX_OUTPUT_CURRENT);
}

void Motor6020::SetSpeedFilter(float ratio) {
    input_speed_filter_ = ratio;
}

}  // namespace driver
