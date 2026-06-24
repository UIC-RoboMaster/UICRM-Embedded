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

#include "MotorCanBase.h"

#include "arm_math.h"
#include "bsp_error_handler.h"
#include "bsp_os.h"
#include "utils.h"

using namespace bsp;

namespace driver {

MotorCANBase::MotorCANBase(bsp::CAN* can, uint16_t rx_id, uint32_t online_threshold)
    : ConnectionDriver(online_threshold),
      theta_(0),
      omega_(0),
      output_shaft_theta_(0),
      output_shaft_omega_(0),
      can_(can),
      rx_id_(rx_id) {
    power_on_angle_ = -1;  // Wait for Update to initialize
    relative_angle_ = 0;
    cumulated_rad_ = 0;
    output_relative_angle_ = 0;
    output_cumulated_turns_ = 0;
    inner_wrap_detector_ = new FloatEdgeDetector(0, PI);
    outer_wrap_detector_ = new FloatEdgeDetector(0, PI);

    enable_ = true;

    // Check if the high resolution timer is initialized
    RM_ASSERT_TRUE(bsp::GetHighresTickMicroSec() != 0, "Highres timer not initialized");
}

void MotorCANBase::UpdateData(const uint8_t* data) {
    UNUSED(data);

    // 上电第一次获取到角度，则记录下来
    if (power_on_angle_ < 0)
        power_on_angle_ = theta_;

    // 如果电机角度从接近 2PI 跳到接近 0，则回绕检测器将检测到下降沿，这意味着电机在穿过编码器边界时正向正方向转动。
    // 反之亦然，电机角度从接近 0 跃升至接近 2PI

    relative_angle_ = theta_ - power_on_angle_;
    if (transmission_ratio_ == 1) {
        output_relative_angle_ = wrap<float>(relative_angle_, 0, 2 * PI);
    } else {
        // 电机屁股的角度给到边界检测器
        inner_wrap_detector_->input(relative_angle_);

        // 记录编码器累计转过几圈
        if (inner_wrap_detector_->negEdge())
            cumulated_rad_ += 2 * PI / transmission_ratio_;
        else if (inner_wrap_detector_->posEdge())
            cumulated_rad_ -= 2 * PI / transmission_ratio_;
        cumulated_rad_ =
            wrap<float>(cumulated_rad_, 0, transmission_ratio_ * 2 * PI);

        output_relative_angle_ =
            wrap<float>(cumulated_rad_ + relative_angle_ / transmission_ratio_, 0, 2 * PI);
    }

    // 输出轴角度给到边界检测器
    outer_wrap_detector_->input(output_relative_angle_);

    // 绝对模式认为输出轴只有一圈，不累计圈数
    if (!absolute_mode_) {
        // 记录输出轴累计转过几圈
        if (outer_wrap_detector_->negEdge())
            output_cumulated_turns_ += 2 * PI;
        else if (outer_wrap_detector_->posEdge())
            output_cumulated_turns_ -= 2 * PI;
    }

    output_shaft_theta_ = output_relative_angle_ + output_cumulated_turns_;
    output_shaft_omega_ = omega_ / transmission_ratio_;

    Heartbeat();

    // 通知子类更新保持状态
    UpdateHoldingState();
}

void MotorCANBase::UpdateHoldingState() {
    // 基类空实现，由子类按需重写
}

float MotorCANBase::GetTheta() const {
    return theta_;
}

float MotorCANBase::GetThetaDelta(float target) const {
    return wrap<float>(target - theta_, -PI, PI);
}

float MotorCANBase::GetOmega() const {
    return omega_;
}

float MotorCANBase::GetOmegaDelta(float target) const {
    return target - omega_;
}

float MotorCANBase::GetOutputShaftTheta() const {
    return output_shaft_theta_;
}

float MotorCANBase::GetOutputShaftOmega() const {
    return output_shaft_omega_;
}

int16_t MotorCANBase::GetCurr() const {
    return 0;
}

uint16_t MotorCANBase::GetTemp() const {
    return 0;
}

void MotorCANBase::SetTransmissionRatio(float ratio) {
    // 设置电机的传动比
    // 这里的传动比不是电机的实际传动比，而是电机与编码器的传动比
    transmission_ratio_ = ratio;
}

void MotorCANBase::Enable() {
    enable_ = true;
}

void MotorCANBase::Disable() {
    enable_ = false;
}

bool MotorCANBase::IsEnable() const {
    return enable_;
}

void MotorCANBase::SetAbsoluteMode(bool enable) {
    absolute_mode_ = enable;
}

}  // namespace driver
