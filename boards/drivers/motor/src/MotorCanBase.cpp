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

using namespace bsp;

namespace driver {

CanMotorBase::CanMotorBase(uint32_t online_threshold) : ConnectionDriver(online_threshold) {
    inner_wrap_detector_ = new FloatEdgeDetector(0, PI);
    outer_wrap_detector_ = new FloatEdgeDetector(0, PI);
    RM_ASSERT_TRUE(GetHighresTickMicroSec() != 0, "Highres timer not initialized");
}

CanMotorBase::~CanMotorBase() {
    delete inner_wrap_detector_;
    delete outer_wrap_detector_;
}

int16_t CanMotorBase::GetOutput() {
    return MotorBase::GetOutput();
}

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

void CanMotorBase::ProcessAngleTracking(AngleTrackingContext ctx) {
    // 如果是第一次接收到数据，初始化 power_on_angle_
    if (ctx.power_on_angle < 0) {
        ctx.power_on_angle = ctx.theta;
    }
    // 计算相对角度和输出轴角度 
    ctx.relative_angle = ctx.theta - ctx.power_on_angle;

    if (ctx.transmission_ratio == 1) {
        ctx.output_relative_angle = wrap<float>(ctx.relative_angle, 0, 2 * PI);
    }
    // 如果有减速比，使用内层回绕检测器处理相对角度的回绕
    else {
        inner_wrap_detector_->input(ctx.relative_angle);

        // 正向回绕：编码器从 2PI 回绕到 0，累计圈数 +1
        if (inner_wrap_detector_->negEdge())
            cumulated_rad_ += 2 * PI / ctx.transmission_ratio;
        // 反向回绕：编码器从 0 回绕到 2PI，累计圈数 -1
        else if (inner_wrap_detector_->posEdge())
            cumulated_rad_ -= 2 * PI / ctx.transmission_ratio;

        // 累计角度限制在 [0, 2PI] 范围内
        cumulated_rad_ = wrap<float>(cumulated_rad_, 0, 2 * PI);

        // 得到输出轴的相对角度（减速比换算）
        ctx.output_relative_angle = wrap<float>(
            cumulated_rad_ + ctx.relative_angle / ctx.transmission_ratio, 0, 2 * PI);
    }

    // 处理输出轴角度的回绕
    outer_wrap_detector_->input(ctx.output_relative_angle);

    // 如果是绝对模式，则不进行累计圈数的更新
    // 如果是相对模式，则根据输出轴角度的回绕更新累计圈数
    if (!ctx.absolute_mode) {
        if (outer_wrap_detector_->negEdge())
            ctx.output_cumulated_angle += 2 * PI;
        else if (outer_wrap_detector_->posEdge())
            ctx.output_cumulated_angle -= 2 * PI;
    }

    // 得到单圈绝对值编码器对应输出轴的累计角度和角速度
    ctx.output_shaft_theta = ctx.output_relative_angle + ctx.output_cumulated_angle;
    ctx.output_shaft_omega = ctx.omega / ctx.transmission_ratio;
}

}  // namespace driver
