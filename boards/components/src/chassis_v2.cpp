/*###########################################################
 # Copyright (c) 2026-2027. BNU-HKBU UIC RoboMaster         #
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

/* 为什么会有chassis_v2
 * chassis是一个底盘的控制代码，其可以是多个电机/多个设备共同组成的一个组件。
 * 但是原chassis.cpp还需完成can_bridge双板通信，负责和gimbal的stm32通信。
 * chassis_v2将重构model类型，并且解耦chassis控制和can_bridge通信。
 * 目前还有老车需要原chassis架构，待老车彻底退役后，可以将_v2后缀去除。
 * by M2rt1n
 */

#include "chassis_v2.h"

#include <cmath>

#include "bsp_error_handler.h"
#include "chassis_kinematics.h"
#include "power_limit_new.h"

namespace control {
    chassis_v2::chassis_v2(driver::SuperCap* super_capacitor) : super_capacitor_(super_capacitor) {
    }

    void chassis_v2::Enable() {
        if (!IsReady()) {
            Disable();
            return;
        }
        state_.enabled = true;
        if (super_capacitor_ != nullptr)
            super_capacitor_->Enable();
    }

    void chassis_v2::Disable() {
        state_.enabled = false;
        speed_ = {};
        Stop();
        if (super_capacitor_ != nullptr)
            super_capacitor_->Disable();
    }

    bool chassis_v2::IsEnabled() const {
        return state_.enabled;
    }

    void chassis_v2::UpdatePower(bool enabled, float max_watt, float voltage, uint8_t buffer_percent) {
        power_.enabled = enabled;
        power_.max_watt = max_watt;
        power_.buffer_percent = buffer_percent > 100 ? 100 : buffer_percent;
        if (std::isfinite(voltage) && voltage > 1)
            power_.voltage = voltage;
    }

    void chassis_v2::UpdatePowerVoltage(float voltage) {
        power_.voltage = voltage;
    }

    void chassis_v2::ApplyPowerLimitWrapper(void* args) {
        static_cast<chassis_v2*>(args)->ApplyPowerLimit();
    }

    mecanum::mecanum(const mecanum::config& config, driver::SuperCap* super_capacitor)
        : chassis_v2(super_capacitor), offset_(config.offset), max_motor_speed_(config.max_motor_speed) {
        RM_ASSERT_TRUE(max_motor_speed_ > 0, "Invalid motor speed limit");
        for (int i = 0; i < chassis_v2_wheel::count; ++i) {
            RM_ASSERT_TRUE(config.motors[i] != nullptr, "Missing chassis motor");
            for (int j = 0; j < i; ++j)
                RM_ASSERT_TRUE(config.motors[i] != config.motors[j], "Duplicate chassis motor");
            motors_[i] = config.motors[i];
            motors_[i]->Disable();
        }
        NewPowerLimit::power_param_t params[4] = {
            {0.000931f, 0.000455f, 0.000006f, 39.151850f},
            {0.000931f, 0.000455f, 0.000006f, 39.151850f},
            {0.000931f, 0.000455f, 0.000006f, 39.151850f},
            {0.000931f, 0.000455f, 0.000006f, 39.151850f},
        };
        limiter_ = new NewPowerLimit(params);
        Disable();
    }

    mecanum::~mecanum() {
        Disable();
        delete limiter_;
    }

    bool mecanum::IsReady() const {
        return true;
    }

    bool mecanum::SetSpeed(float x, float y, float turn) {
        // TODO: std::isfinite()
        if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(turn) || !IsReady()) {
            Disable();
            return false;
        }
        Solve({x, y, turn});
        // Preserve old scaling, also bounding offset-amplified wheel targets.
        float peak = std::fabs(x) + std::fabs(y) + std::fabs(turn);
        for (float speed : speeds_) {
            if (!std::isfinite(speed)) {
                Disable();
                return false;
            }
            peak = std::fmax(peak, std::fabs(speed));
        }
        const float scale = peak > max_motor_speed_ ? max_motor_speed_ / peak : 1.0f;
        for (float& speed : speeds_)
            speed *= scale;
        speed_ = {x, y, turn};
        return true;
    }

    void mecanum::Update() {
        if (!state_.enabled || !IsReady()) {
            Disable();
            return;
        }
        if (!SetSpeed(speed_.x, speed_.y, speed_.turn))
            return;
        for (int i = 0; i < chassis_v2_wheel::count; ++i) {
            motors_[i]->SetTarget(speeds_[i]);
            motors_[i]->Enable();
        }
    }

    void mecanum::Stop() {
        state_.enabled = false;
        speed_ = {};
        if (super_capacitor_ != nullptr)
            super_capacitor_->Disable();
        for (float& speed : speeds_){
            speed = 0;
        }
        for (auto* motor : motors_) {
            motor->Disable();
            motor->SetTarget(0);
            motor->SetOutput(0);
        }
    }

    bool mecanum::SetMaxMotorSpeed(float max_speed) {
        if (!std::isfinite(max_speed) || max_speed <= 0 || !IsReady()) {
            Disable();
            return false;
        }
        max_motor_speed_ = max_speed;
        return SetSpeed(speed_.x, speed_.y, speed_.turn);
    }

    void mecanum::Solve(const chassis_speed& command) {
        const auto wheels = kinematics::MecanumInverse(command.x, command.y, command.turn, offset_);
        speeds_[chassis_v2_wheel::front_left] = wheels.front_left;
        speeds_[chassis_v2_wheel::front_right] = wheels.front_right;
        speeds_[chassis_v2_wheel::back_left] = wheels.back_left;
        speeds_[chassis_v2_wheel::back_right] = wheels.back_right;
    }

    void mecanum::ZeroOutputs() {
        for (auto* motor : motors_)
            motor->SetOutput(0);
    }

    void mecanum::ApplyPowerLimit() {
        const chassis_power power = power_;
        const bool enabled = state_.enabled;
        if (!enabled || !IsReady()) {
            ZeroOutputs();
            return;
        }
        if (!power.enabled)
            return;
        if (!std::isfinite(power.voltage) || power.voltage <= 1 || !std::isfinite(power.max_watt) ||
            power.max_watt <= 0) {
            ZeroOutputs();
            return;
        }
        float max_current_ma = power.max_watt / power.voltage * 1000;
        max_current_ma *= linear_remap_clip(static_cast<int>(power.buffer_percent), 20, 80, 0.8f, 1.5f);
        if (!std::isfinite(max_current_ma) || max_current_ma < 1) {
            ZeroOutputs();
            return;
        }
        // The existing limiter accumulates signed int16 currents internally.
        max_current_ma = std::fmin(max_current_ma, 32767.0f);
        int16_t currents[4];
        float omega[4];
        for (int i = 0; i < chassis_v2_wheel::count; ++i) {
            currents[i] = motors_[i]->GetOutput();
            omega[i] = motors_[i]->GetOmega();
            if (!std::isfinite(omega[i])) {
                ZeroOutputs();
                return;
            }
        }
        limiter_->LimitPower(currents, omega, static_cast<uint16_t>(max_current_ma));
        for (int i = 0; i < chassis_v2_wheel::count; ++i)
            motors_[i]->SetOutput(motors_[i]->IsEnable() ? currents[i] : 0);
    }

    omni::omni(const omni::config& config, driver::SuperCap* super_capacitor)
        : chassis_v2(super_capacitor), offset_(config.offset), max_motor_speed_(config.max_motor_speed) {
        RM_ASSERT_TRUE(max_motor_speed_ > 0, "Invalid motor speed limit");
        for (int i = 0; i < chassis_v2_wheel::count; ++i) {
            RM_ASSERT_TRUE(config.motors[i] != nullptr, "Missing chassis motor");
            for (int j = 0; j < i; ++j)
                RM_ASSERT_TRUE(config.motors[i] != config.motors[j], "Duplicate chassis motor");
            motors_[i] = config.motors[i];
            motors_[i]->Disable();
        }
        NewPowerLimit::power_param_t params[4] = {
            {0.000931f, 0.000455f, 0.000006f, 39.151850f},
            {0.000931f, 0.000455f, 0.000006f, 39.151850f},
            {0.000931f, 0.000455f, 0.000006f, 39.151850f},
            {0.000931f, 0.000455f, 0.000006f, 39.151850f},
        };
        limiter_ = new NewPowerLimit(params);
        Disable();
    }

    omni::~omni() {
        Disable();
        delete limiter_;
    }

    bool omni::IsReady() const {
        return true;
    }

    bool omni::SetSpeed(float x, float y, float turn) {
        if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(turn) || !IsReady()) {
            Disable();
            return false;
        }
        Solve({x, y, turn});
        // Preserve old scaling, also bounding offset-amplified wheel targets.
        float peak = std::fabs(x) + std::fabs(y) + std::fabs(turn);
        for (float speed : speeds_) {
            if (!std::isfinite(speed)) {
                Disable();
                return false;
            }
            peak = std::fmax(peak, std::fabs(speed));
        }
        const float scale = peak > max_motor_speed_ ? max_motor_speed_ / peak : 1.0f;
        for (float& speed : speeds_)
            speed *= scale;
        speed_ = {x, y, turn};
        return true;
    }

    void omni::Update() {
        if (!state_.enabled || !IsReady()) {
            Disable();
            return;
        }
        if (!SetSpeed(speed_.x, speed_.y, speed_.turn))
            return;
        for (int i = 0; i < chassis_v2_wheel::count; ++i) {
            motors_[i]->SetTarget(speeds_[i]);
            motors_[i]->Enable();
        }
    }

    void omni::Stop() {
        state_.enabled = false;
        speed_ = {};
        if (super_capacitor_ != nullptr)
            super_capacitor_->Disable();
        for (float& speed : speeds_)
            speed = 0;
        for (auto* motor : motors_) {
            motor->Disable();
            motor->SetTarget(0);
            motor->SetOutput(0);
        }
    }

    bool omni::SetMaxMotorSpeed(float max_speed) {
        if (!std::isfinite(max_speed) || max_speed <= 0 || !IsReady()) {
            Disable();
            return false;
        }
        max_motor_speed_ = max_speed;
        return SetSpeed(speed_.x, speed_.y, speed_.turn);
    }

    void omni::Solve(const chassis_speed& command) {
        const auto wheels = kinematics::OmniInverse(command.x, command.y, command.turn, offset_);
        speeds_[chassis_v2_wheel::front_left] = wheels.front_left;
        speeds_[chassis_v2_wheel::front_right] = wheels.front_right;
        speeds_[chassis_v2_wheel::back_left] = wheels.back_left;
        speeds_[chassis_v2_wheel::back_right] = wheels.back_right;
    }

    void omni::ZeroOutputs() {
        for (auto* motor : motors_)
            motor->SetOutput(0);
    }

    void omni::ApplyPowerLimit() {
        const uint32_t primask = __get_PRIMASK();
        __disable_irq();
        const chassis_power power = power_;
        const bool enabled = state_.enabled;
        __set_PRIMASK(primask);
        if (!enabled || !IsReady()) {
            ZeroOutputs();
            return;
        }
        if (!power.enabled)
            return;
        if (!std::isfinite(power.voltage) || power.voltage <= 1 || !std::isfinite(power.max_watt) ||
            power.max_watt <= 0) {
            ZeroOutputs();
            return;
        }
        float max_current_ma = power.max_watt / power.voltage * 1000;
        max_current_ma *= linear_remap_clip(static_cast<int>(power.buffer_percent), 20, 80, 0.8f, 1.5f);
        if (!std::isfinite(max_current_ma) || max_current_ma < 1) {
            ZeroOutputs();
            return;
        }
        // The existing limiter accumulates signed int16 currents internally.
        max_current_ma = std::fmin(max_current_ma, 32767.0f);
        int16_t currents[4];
        float omega[4];
        for (int i = 0; i < chassis_v2_wheel::count; ++i) {
            currents[i] = motors_[i]->GetOutput();
            omega[i] = motors_[i]->GetOmega();
            if (!std::isfinite(omega[i])) {
                ZeroOutputs();
                return;
            }
        }
        limiter_->LimitPower(currents, omega, static_cast<uint16_t>(max_current_ma));
        for (int i = 0; i < chassis_v2_wheel::count; ++i)
            motors_[i]->SetOutput(motors_[i]->IsEnable() ? currents[i] : 0);
    }

    // TODO: implement this model after its mechanics, units and actuators are defined.
    bool steer::IsReady() const {
        return false;
    }
    bool steer::SetSpeed(float, float, float) {
        Disable();
        return false;
    }
    void steer::Update() {
        Disable();
    }
    void steer::Stop() {
        state_.enabled = false;
        speed_ = {};
        if (super_capacitor_ != nullptr)
            super_capacitor_->Disable();
    }
    bool steer::SetMaxMotorSpeed(float) {
        Disable();
        return false;
    }
    void steer::ApplyPowerLimit() {
    }

    // TODO: implement this model after its mechanics, units and actuators are defined.
    bool swerve::IsReady() const {
        return false;
    }
    bool swerve::SetSpeed(float, float, float) {
        Disable();
        return false;
    }
    void swerve::Update() {
        Disable();
    }
    void swerve::Stop() {
        state_.enabled = false;
        speed_ = {};
        if (super_capacitor_ != nullptr)
            super_capacitor_->Disable();
    }
    bool swerve::SetMaxMotorSpeed(float) {
        Disable();
        return false;
    }
    void swerve::ApplyPowerLimit() {
    }

}  // namespace control
