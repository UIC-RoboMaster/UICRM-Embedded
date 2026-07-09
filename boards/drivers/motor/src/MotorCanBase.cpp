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

    // 编码器单圈内角度 [0, 2π]（相对上电位置）
    ctx.encoder_relative_angle = wrap<float>(ctx.theta - ctx.power_on_angle, 0, 2 * PI);

    // 在 raw theta 上检测 2π↔0 回绕，累计编码器圈数
    inner_wrap_detector_->input(ctx.theta);
    // 正向回绕：编码器从 2π 回绕到 0，累计圈数 +1
    if (inner_wrap_detector_->negEdge())
        ctx.encoder_cumulated_turns += 1;
    // 反向回绕：编码器从 0 回绕到 2π，累计圈数 -1
    else if (inner_wrap_detector_->posEdge())
        ctx.encoder_cumulated_turns -= 1;

    // 编码器累计角度 = 累计圈数 × 2π + 圈内角，连续无跳变
    ctx.encoder_cumulated_angle = ctx.encoder_cumulated_turns * 2 * PI + ctx.encoder_relative_angle;

    // 输出轴圈内角 [0, 2π]（由编码器累计角度经减速比换算）
    ctx.output_relative_angle =
        wrap<float>(ctx.encoder_cumulated_angle / ctx.transmission_ratio, 0, 2 * PI);

    // 输出轴回绕检测，累计输出轴圈数
    outer_wrap_detector_->input(ctx.output_relative_angle);
    if (outer_wrap_detector_->negEdge())
        ctx.output_cumulated_turns += 1;
    else if (outer_wrap_detector_->posEdge())
        ctx.output_cumulated_turns -= 1;

    // 输出轴多圈累计角度（内部状态，absolute 模式下仍持续累计）
    ctx.output_cumulated_angle = ctx.output_cumulated_turns * 2 * PI + ctx.output_relative_angle;

    // absolute 模式：对外输出限制在 [0, 2π]；相对模式：输出多圈累计角
    ctx.output_shaft_theta = ctx.absolute_mode ? ctx.output_relative_angle : ctx.output_cumulated_angle;
    ctx.output_shaft_omega = ctx.omega / ctx.transmission_ratio;
}

void CanMotorBase::FinishFeedbackUpdate(AngleTrackingContext ctx) {
    ProcessAngleTracking(ctx);
    Heartbeat();
}

void CanMotorBase::TransmitFrame(bsp::CAN* can, uint16_t tx_id, const uint8_t data[8], uint8_t dlc) {
    can->Transmit(tx_id, data, dlc);
}

}  // namespace driver
