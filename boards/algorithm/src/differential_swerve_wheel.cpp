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

#include "differential_swerve_wheel.h"

#include <cmath>

namespace control {

    DifferentialSwerveWheel::DifferentialSwerveWheel(const Config& config)
        : config_(config), yaw_pid_(config.yaw_pid_init) {
        RefreshInverse();
        Reset();
        has_feedback_ = false;
    }

    void DifferentialSwerveWheel::Reset(float motor1_theta, float motor2_theta) {
        has_feedback_ = true;
        last_motor1_theta_ = motor1_theta;
        last_motor2_theta_ = motor2_theta;

        state_.motor1_theta = motor1_theta;
        state_.motor2_theta = motor2_theta;
        state_.motor1_omega = 0.0f;
        state_.motor2_omega = 0.0f;
        state_.yaw_rate = 0.0f;
        state_.drive_speed = 0.0f;

        yaw_zero_ = config_.kinematics.a11 * motor1_theta + config_.kinematics.a12 * motor2_theta;
        drive_zero_ =
            config_.kinematics.a21 * motor1_theta + config_.kinematics.a22 * motor2_theta;
        state_.yaw_angle = 0.0f;
        state_.drive_angle = 0.0f;

        yaw_target_ = 0.0f;
        drive_speed_target_ = 0.0f;

        motor_target_.motor1_theta = motor1_theta;
        motor_target_.motor2_theta = motor2_theta;
        motor_target_.motor1_omega = 0.0f;
        motor_target_.motor2_omega = 0.0f;

        yaw_pid_.Reset();
        yaw_pid_.ResetIntegral();

        dwt_ready_ = false;
        last_dwt_cnt_ = 0;
    }

    void DifferentialSwerveWheel::UpdateFeedback(float motor1_theta, float motor2_theta) {
        if (!has_feedback_) {
            Reset(motor1_theta, motor2_theta);
            return;
        }
        UpdateStateFromFeedback(motor1_theta, motor2_theta, GetDeltaT());
    }

    DifferentialSwerveWheel::MotorTarget DifferentialSwerveWheel::Update(float motor1_theta,
                                                                         float motor2_theta) {
        if (!has_feedback_) {
            Reset(motor1_theta, motor2_theta);
            return motor_target_;
        }

        const float dt = GetDeltaT();
        UpdateStateFromFeedback(motor1_theta, motor2_theta, dt);

        if (!invertible_) {
            motor_target_.motor1_theta = state_.motor1_theta;
            motor_target_.motor2_theta = state_.motor2_theta;
            motor_target_.motor1_omega = 0.0f;
            motor_target_.motor2_omega = 0.0f;
            return motor_target_;
        }

        const float wrapped_yaw_target =
            state_.yaw_angle + wrap<float>(yaw_target_ - state_.yaw_angle, -PI, PI);
        float yaw_rate_cmd = yaw_pid_.ComputeOutput(wrapped_yaw_target, state_.yaw_angle);
        yaw_rate_cmd = clip<float>(yaw_rate_cmd, -config_.max_yaw_rate, config_.max_yaw_rate);

        const float drive_speed_cmd =
            clip<float>(drive_speed_target_, -config_.max_drive_speed, config_.max_drive_speed);

        float motor1_omega = 0.0f;
        float motor2_omega = 0.0f;
        SolveMotorOmega(yaw_rate_cmd, drive_speed_cmd, motor1_omega, motor2_omega);

        const float max_abs_motor_omega =
            max<float>(std::fabs(motor1_omega), std::fabs(motor2_omega));
        if (max_abs_motor_omega > config_.max_motor_speed &&
            config_.max_motor_speed > config_.singular_epsilon) {
            const float scale = config_.max_motor_speed / max_abs_motor_omega;
            motor1_omega *= scale;
            motor2_omega *= scale;
        }

        motor_target_.motor1_omega = motor1_omega;
        motor_target_.motor2_omega = motor2_omega;
        motor_target_.motor1_theta += motor1_omega * dt;
        motor_target_.motor2_theta += motor2_omega * dt;
        return motor_target_;
    }

    void DifferentialSwerveWheel::SetTarget(float yaw_relative, float drive_speed) {
        SetYawTarget(yaw_relative);
        SetDriveSpeed(drive_speed);
    }

    void DifferentialSwerveWheel::SetYawTarget(float yaw_relative) {
        yaw_target_ = yaw_relative;
    }

    void DifferentialSwerveWheel::SetDriveSpeed(float drive_speed) {
        drive_speed_target_ = drive_speed;
    }

    void DifferentialSwerveWheel::SetKinematics(const Matrix2x2& kinematics) {
        config_.kinematics = kinematics;
        RefreshInverse();

        if (has_feedback_) {
            yaw_zero_ = config_.kinematics.a11 * state_.motor1_theta +
                        config_.kinematics.a12 * state_.motor2_theta - state_.yaw_angle;
            drive_zero_ = config_.kinematics.a21 * state_.motor1_theta +
                          config_.kinematics.a22 * state_.motor2_theta - state_.drive_angle;
        }
    }

    void DifferentialSwerveWheel::ReInitYawPID(const ConstrainedPID::PID_Init_t& pid_init) {
        config_.yaw_pid_init = pid_init;
        yaw_pid_.Reinit(pid_init);
        yaw_pid_.Reset();
        yaw_pid_.ResetIntegral();
    }

    void DifferentialSwerveWheel::SetMaxYawRate(float max_yaw_rate) {
        config_.max_yaw_rate = max_yaw_rate;
    }

    void DifferentialSwerveWheel::SetMaxDriveSpeed(float max_drive_speed) {
        config_.max_drive_speed = max_drive_speed;
    }

    void DifferentialSwerveWheel::SetMaxMotorSpeed(float max_motor_speed) {
        config_.max_motor_speed = max_motor_speed;
    }

    const DifferentialSwerveWheel::MotorTarget& DifferentialSwerveWheel::GetMotorTarget() const {
        return motor_target_;
    }

    const DifferentialSwerveWheel::State& DifferentialSwerveWheel::GetState() const {
        return state_;
    }

    float DifferentialSwerveWheel::GetYawTarget() const {
        return yaw_target_;
    }

    float DifferentialSwerveWheel::GetDriveSpeedTarget() const {
        return drive_speed_target_;
    }

    bool DifferentialSwerveWheel::IsKinematicsInvertible() const {
        return invertible_;
    }

    void DifferentialSwerveWheel::UpdateStateFromFeedback(float motor1_theta, float motor2_theta,
                                                          float dt) {
        const float bounded_dt = clip<float>(dt, config_.min_dt, config_.max_dt);

        state_.motor1_omega = (motor1_theta - last_motor1_theta_) / bounded_dt;
        state_.motor2_omega = (motor2_theta - last_motor2_theta_) / bounded_dt;
        state_.motor1_theta = motor1_theta;
        state_.motor2_theta = motor2_theta;

        state_.yaw_angle =
            config_.kinematics.a11 * motor1_theta + config_.kinematics.a12 * motor2_theta -
            yaw_zero_;
        state_.drive_angle =
            config_.kinematics.a21 * motor1_theta + config_.kinematics.a22 * motor2_theta -
            drive_zero_;

        state_.yaw_rate = config_.kinematics.a11 * state_.motor1_omega +
                          config_.kinematics.a12 * state_.motor2_omega;
        state_.drive_speed = config_.kinematics.a21 * state_.motor1_omega +
                             config_.kinematics.a22 * state_.motor2_omega;

        last_motor1_theta_ = motor1_theta;
        last_motor2_theta_ = motor2_theta;
    }

    void DifferentialSwerveWheel::SolveMotorOmega(float yaw_rate, float drive_speed,
                                                  float& motor1_omega,
                                                  float& motor2_omega) const {
        if (!invertible_) {
            motor1_omega = 0.0f;
            motor2_omega = 0.0f;
            return;
        }

        motor1_omega = inverse_.a11 * yaw_rate + inverse_.a12 * drive_speed;
        motor2_omega = inverse_.a21 * yaw_rate + inverse_.a22 * drive_speed;
    }

    float DifferentialSwerveWheel::GetDeltaT() {
        if (!dwt_ready_) {
            last_dwt_cnt_ = DWT->CYCCNT;
            dwt_ready_ = true;
            return config_.min_dt;
        }
        return DWT_GetDeltaT(&last_dwt_cnt_);
    }

    void DifferentialSwerveWheel::RefreshInverse() {
        const float det = config_.kinematics.a11 * config_.kinematics.a22 -
                          config_.kinematics.a12 * config_.kinematics.a21;
        invertible_ = std::fabs(det) > config_.singular_epsilon;

        if (!invertible_) {
            inverse_ = {};
            return;
        }

        const float inv_det = 1.0f / det;
        inverse_.a11 = config_.kinematics.a22 * inv_det;
        inverse_.a12 = -config_.kinematics.a12 * inv_det;
        inverse_.a21 = -config_.kinematics.a21 * inv_det;
        inverse_.a22 = config_.kinematics.a11 * inv_det;
    }

}  // namespace control
