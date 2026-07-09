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
    : DjiMotorBase(can, rx_id, tx_id != 0x00 ? tx_id : ResolveTxId(rx_id)) {
    if (tx_id == 0x00) {
        RM_ASSERT_GE(rx_id, 0x205, "Invalid rx id for GM6020");
    }
    state_.transmission_ratio = Motor6020Config::ORIGINAL_TRANSMISSION_RATIO;
    torque_constant_ = Motor6020Config::RATED_TORQUE_CONSTANT;
    max_current_amp_ = Motor6020Config::MAX_CURRENT;
    max_raw_current_ = Motor6020Config::MAX_RAW_CURRENT;
    // GM6020 使用绝对值编码器，直接将上电角度初始化为 0
    state_.power_on_angle = 0;
    CanMotorBase::RegisterCanCallback(can, rx_id, &Motor6020::RxThunk, this);
}

void Motor6020::RxThunk(void* ctx, const uint8_t data[]) {
    static_cast<Motor6020*>(ctx)->UpdateData(data);
}

void Motor6020::UpdateData(const uint8_t data[]) {
    state_.raw_theta = data[0] << 8 | data[1];
    state_.raw_omega = data[2] << 8 | data[3];
    state_.raw_current = (int16_t)(data[4] << 8 | data[5]);
    state_.raw_temperature = data[6];

    // GM6020 转子机械角度值范围为 0~8191
    // 映射 theta 角度为 0~2PI
    state_.theta = linear_remap<int16_t, float>(state_.raw_theta, 0, Motor6020Config::MAX_RAW_THETA, 0.0f, 2 * PI);
    // GM6020 转子转速值单位为 rpm，rad/s = rpm * 2 * PI / 60
    // 映射 omega 角速度为 rad/s
    state_.omega = (state_.raw_omega * 2 * PI / 60) * input_speed_filter_
                 + state_.omega * (1 - input_speed_filter_);
    // GM6020 转矩电流反馈 raw_current ∈ [-16384, 16384] 对应 [-3A, 3A]
    state_.current = linear_remap<int16_t, float>(state_.raw_current, -Motor6020Config::MAX_RAW_CURRENT, Motor6020Config::MAX_RAW_CURRENT,
                                  -Motor6020Config::MAX_CURRENT, Motor6020Config::MAX_CURRENT);
    state_.torque = state_.current * torque_constant_;

    state_.feedback_pending = true;
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
    output_ = clip<int16_t>(val, -Motor6020Config::MAX_RAW_CURRENT, Motor6020Config::MAX_RAW_CURRENT);
}

void Motor6020::SetSpeedFilter(float ratio) {
    input_speed_filter_ = ratio;
}

}  // namespace driver
