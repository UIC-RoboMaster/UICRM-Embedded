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

#pragma once

#include "MotorBase.h"
#include "arm_math.h"
#include "bsp_can.h"
#include "bsp_error_handler.h"
#include "bsp_os.h"
#include "connection_driver.h"
#include "utils.h"

namespace driver {

/**
 * @brief CAN 通信电机的 CRTP 抽象基类
 *
 * 通过模板参数 Derived 访问子类的 state_ 结构体，提供所有 CAN 总线电机共用的能力：
 * - 编码器角度/速度的获取（非虚，编译期绑定，零开销）
 * - 传动比换算（输出轴角度/速度）
 * - 编码器回绕检测与多圈累计
 * - 连接心跳检测
 *
 * 子类需：
 *  1. 提供 state_ 成员（包含 theta, omega, output_shaft_theta, enable, can 等字段）
 *  2. 声明 friend class MotorCANBase<Derived>;
 */
template <typename Derived>
class MotorCANBase : public MotorBase, public ConnectionDriver {
  public:
    MotorCANBase(uint32_t online_threshold = 30) : ConnectionDriver(online_threshold) {
        inner_wrap_detector_ = new FloatEdgeDetector(0, PI);
        outer_wrap_detector_ = new FloatEdgeDetector(0, PI);
        RM_ASSERT_TRUE(bsp::GetHighresTickMicroSec() != 0, "Highres timer not initialized");
    }

    virtual void UpdateData(const uint8_t data[]) { (void)data; }
    virtual void UpdateHoldingState() {}
    virtual void PrintData() const = 0;
    virtual void SetTarget(float target, bool override = true) {
        (void)target;
        (void)override;
    }

    // ── 非虚 getter（CRTP 编译期绑定）──
    float GetTheta() const { return state().theta; }
    float GetOmega() const { return state().omega; }
    float GetOutputShaftTheta() const { return state().output_shaft_theta; }
    float GetOutputShaftOmega() const { return state().output_shaft_omega; }
    float GetThetaDelta(const float target) const {
        return wrap<float>(target - state().theta, -PI, PI);
    }
    float GetOmegaDelta(const float target) const {
        return target - state().omega;
    }
    int16_t GetCurr() const { return state().raw_current; }
    uint16_t GetTemp() const { return state().raw_temperature; }

    void SetTransmissionRatio(float ratio) { state().transmission_ratio = ratio; }
    void Enable() { state().enable = true; }
    void Disable() { state().enable = false; }
    bool IsEnable() const { return state().enable; }
    void SetAbsoluteMode(bool enable) { state().absolute_mode = enable; }

    // ── CAN 通信基础封装 ──
    /**
     * @brief 向 CAN 总线发送一帧数据
     * @param data 8 字节数据帧
     */
    void SendPacket(const uint8_t data[8]) {
        state().can->Transmit(state().tx_id, data, 8);
    }

  protected:
    /**
     * @brief 注册 CAN 接收回调
     * @note 子类构造函数中调用一次即可，自动绑定到所属 CAN 的 rx_id
     */
    void RegisterCanCallback() {
        state().can->RegisterRxCallback(state().rx_id, &MotorCANBase::CanRxCallback, this);
    }

  private:
    /**
     * @brief 静态 CAN 接收回调，转发到子类 UpdateData
     */
    static void CanRxCallback(const uint8_t data[], void* args) {
        static_cast<Derived*>(args)->UpdateData(data);
    }

  protected:
    /**
     * @brief 角度追踪处理
     * @note 子类在 UpdateData 中解析完协议后调用此方法
     */
    void ProcessAngleTracking() {
        auto& motor_state = state();

        if (motor_state.power_on_angle < 0)
            motor_state.power_on_angle = motor_state.theta;

        motor_state.relative_angle = motor_state.theta - motor_state.power_on_angle;
        if (motor_state.transmission_ratio == 1) {
            motor_state.output_relative_angle = wrap<float>(motor_state.relative_angle, 0, 2 * PI);
        } else {
            inner_wrap_detector_->input(motor_state.relative_angle);

            if (inner_wrap_detector_->negEdge())
                motor_state.cumulated_rad += 2 * PI / motor_state.transmission_ratio;
            else if (inner_wrap_detector_->posEdge())
                motor_state.cumulated_rad -= 2 * PI / motor_state.transmission_ratio;
            motor_state.cumulated_rad =
                wrap<float>(motor_state.cumulated_rad, 0, motor_state.transmission_ratio * 2 * PI);

            motor_state.output_relative_angle = wrap<float>(
                motor_state.cumulated_rad + motor_state.relative_angle / motor_state.transmission_ratio, 0, 2 * PI);
        }

        outer_wrap_detector_->input(motor_state.output_relative_angle);

        if (!motor_state.absolute_mode) {
            if (outer_wrap_detector_->negEdge())
                motor_state.output_cumulated_turns += 2 * PI;
            else if (outer_wrap_detector_->posEdge())
                motor_state.output_cumulated_turns -= 2 * PI;
        }

        motor_state.output_shaft_theta = motor_state.output_relative_angle + motor_state.output_cumulated_turns;
        motor_state.output_shaft_omega = motor_state.omega / motor_state.transmission_ratio;

        Heartbeat();
        UpdateHoldingState();
    }

    // CRTP 访问子类 state_
    Derived& derived() { return *static_cast<Derived*>(this); }
    const Derived& derived() const { return *static_cast<const Derived*>(this); }
    auto& state() { return derived().state_; }
    const auto& state() const { return derived().state_; }

    FloatEdgeDetector* inner_wrap_detector_;
    FloatEdgeDetector* outer_wrap_detector_;
};

}  // namespace driver
