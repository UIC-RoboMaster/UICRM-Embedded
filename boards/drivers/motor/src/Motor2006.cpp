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
    : DjiMotorBase(can, rx_id, tx_id != 0x00 ? tx_id : ResolveTxId(rx_id)) {
    // M2006 RX_ID = 0x200 + 电调 ID
    // TX_ID 须在 DjiMotorBase 构造前解析，否则分组发送会使用错误的 CAN ID
    if (tx_id == 0x00) {
        RM_ASSERT_GE(rx_id, 0x201, "Invalid rx id for M2006");
    }
    state_.transmission_ratio = Motor2006Config::ORIGINAL_TRANSMISSION_RATIO;
    torque_constant_ = Motor2006Config::RATED_TORQUE_CONSTANT;
    max_current_amp_ = Motor2006Config::MAX_CURRENT;
    max_raw_current_ = Motor2006Config::MAX_RAW_CURRENT;
    CanMotorBase::RegisterCanCallback(can, rx_id, &Motor2006::RxThunk, this);
}

void Motor2006::RxThunk(void* ctx, const uint8_t data[]) {
    static_cast<Motor2006*>(ctx)->UpdateData(data);
}

void Motor2006::UpdateData(const uint8_t data[]) {
    state_.raw_theta = data[0] << 8 | data[1];
    state_.raw_omega = data[2] << 8 | data[3];
    state_.raw_current = (int16_t)(data[4] << 8 | data[5]);

    // M2006 转子机械角度值范围为 0~8191
    // 映射 theta 角度为 0~2PI
    state_.theta = linear_remap<int16_t, float>(state_.raw_theta, 0, Motor2006Config::MAX_RAW_THETA, 0.0f, 2 * PI);
    // 转子转速值单位为 rpm，rad/s = rpm * 2 * PI / 60
    // 映射 omega 角速度为 rad/s
    state_.omega = state_.raw_omega * 2 * PI / 60;
    // C610 转矩电流反馈 raw_current ∈ [-10000, 10000] 对应 [-10A, 10A]
    state_.current = linear_remap<int16_t, float>(state_.raw_current, -Motor2006Config::MAX_RAW_CURRENT,
                                                  Motor2006Config::MAX_RAW_CURRENT, -Motor2006Config::MAX_CURRENT,
                                                  Motor2006Config::MAX_CURRENT);

    state_.feedback_pending = true;
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
    output_ = clip<int16_t>(val, -Motor2006Config::MAX_RAW_CURRENT, Motor2006Config::MAX_RAW_CURRENT);
}

}  // namespace driver
