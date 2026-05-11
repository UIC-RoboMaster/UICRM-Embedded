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

namespace control {

    namespace {
        constexpr float kSingularEpsilon = 1e-6f;
        constexpr float kMinDt = 1e-4f;
    }

    DifferentialSwerveWheelKinematic::DifferentialSwerveWheelKinematic(
        const Matrix2x2& control_matrix)
        : control_matrix_(control_matrix),
          inverse_matrix_(Inverse(control_matrix)),
          invertible_(Determinant(control_matrix) > kSingularEpsilon ||
                      Determinant(control_matrix) < -kSingularEpsilon),
          state_({0.0f, 0.0f, 0.0f}),
          target_drive_angle_(0.0f),
          last_feedback_dwt_cnt_(0),
          last_target_dwt_cnt_(0),
          feedback_dwt_ready_(false),
          target_dwt_ready_(false) {
    }

    DifferentialSwerveWheelKinematic::Vector2 DifferentialSwerveWheelKinematic::InverseSolve(
        float yaw_rate, float drive_speed) const {
        Vector2 result = {0.0f, 0.0f};
        InverseSolve(yaw_rate, drive_speed, result.x1, result.x2);
        return result;
    }

    void DifferentialSwerveWheelKinematic::InverseSolve(float yaw_rate, float drive_speed,
                                                        float& motor1_omega,
                                                        float& motor2_omega) const {
        if (!invertible_) {
            motor1_omega = 0.0f;
            motor2_omega = 0.0f;
            return;
        }

        motor1_omega = inverse_matrix_.a11 * yaw_rate + inverse_matrix_.a12 * drive_speed;
        motor2_omega = inverse_matrix_.a21 * yaw_rate + inverse_matrix_.a22 * drive_speed;
    }

    DifferentialSwerveWheelKinematic::Vector2 DifferentialSwerveWheelKinematic::ForwardSolve(
        float motor1_omega, float motor2_omega) const {
        Vector2 result = {0.0f, 0.0f};
        ForwardSolve(motor1_omega, motor2_omega, result.x1, result.x2);
        return result;
    }

    void DifferentialSwerveWheelKinematic::ForwardSolve(float motor1_omega, float motor2_omega,
                                                        float& yaw_rate,
                                                        float& drive_speed) const {
        yaw_rate = control_matrix_.a11 * motor1_omega + control_matrix_.a12 * motor2_omega;
        drive_speed = control_matrix_.a21 * motor1_omega + control_matrix_.a22 * motor2_omega;
    }

    DifferentialSwerveWheelKinematic::WheelState DifferentialSwerveWheelKinematic::Update(
        float motor1_theta, float motor2_theta) {
        const float yaw_angle =
            control_matrix_.a11 * motor1_theta + control_matrix_.a12 * motor2_theta;
        const float drive_angle =
            control_matrix_.a21 * motor1_theta + control_matrix_.a22 * motor2_theta;

        float drive_speed = 0.0f;
        if (feedback_dwt_ready_) {
            float dt = DWT_GetDeltaT(&last_feedback_dwt_cnt_);
            if (dt < kMinDt) {
                dt = kMinDt;
            }
            drive_speed = (drive_angle - state_.drive_angle) / dt;
        } else {
            last_feedback_dwt_cnt_ = DWT->CYCCNT;
            feedback_dwt_ready_ = true;
        }

        state_.yaw_angle = yaw_angle;
        state_.drive_angle = drive_angle;
        state_.drive_speed = drive_speed;
        return state_;
    }

    void DifferentialSwerveWheelKinematic::Update(float motor1_theta, float motor2_theta,
                                                  float& yaw_angle, float& drive_speed) {
        const WheelState state = Update(motor1_theta, motor2_theta);
        yaw_angle = state.yaw_angle;
        drive_speed = state.drive_speed;
    }

    void DifferentialSwerveWheelKinematic::Reset(float motor1_theta, float motor2_theta) {
        state_.yaw_angle = control_matrix_.a11 * motor1_theta + control_matrix_.a12 * motor2_theta;
        state_.drive_angle =
            control_matrix_.a21 * motor1_theta + control_matrix_.a22 * motor2_theta;
        state_.drive_speed = 0.0f;
        target_drive_angle_ = state_.drive_angle;
        last_feedback_dwt_cnt_ = DWT->CYCCNT;
        last_target_dwt_cnt_ = DWT->CYCCNT;
        feedback_dwt_ready_ = true;
        target_dwt_ready_ = true;
    }

    DifferentialSwerveWheelKinematic::MotorTarget
    DifferentialSwerveWheelKinematic::UpdateTarget(float yaw_angle_target, float drive_speed_target) {
        MotorTarget result = {0.0f, 0.0f};
        UpdateTarget(yaw_angle_target, drive_speed_target, result.motor1_angle, result.motor2_angle);
        return result;
    }

    void DifferentialSwerveWheelKinematic::UpdateTarget(float yaw_angle_target,
                                                        float drive_speed_target,
                                                        float& motor1_angle,
                                                        float& motor2_angle) {
        float dt = kMinDt;
        if (target_dwt_ready_) {
            dt = DWT_GetDeltaT(&last_target_dwt_cnt_);
            if (dt < kMinDt) {
                dt = kMinDt;
            }
        } else {
            last_target_dwt_cnt_ = DWT->CYCCNT;
            target_dwt_ready_ = true;
        }

        target_drive_angle_ += drive_speed_target * dt;

        if (!invertible_) {
            motor1_angle = 0.0f;
            motor2_angle = 0.0f;
            return;
        }

        motor1_angle = inverse_matrix_.a11 * yaw_angle_target +
                       inverse_matrix_.a12 * target_drive_angle_;
        motor2_angle = inverse_matrix_.a21 * yaw_angle_target +
                       inverse_matrix_.a22 * target_drive_angle_;
    }

    void DifferentialSwerveWheelKinematic::ResetTarget(float yaw_angle_target,
                                                       float drive_angle_target) {
        target_drive_angle_ = drive_angle_target;

        state_.yaw_angle = yaw_angle_target;
        state_.drive_angle = drive_angle_target;
        state_.drive_speed = 0.0f;
        last_feedback_dwt_cnt_ = DWT->CYCCNT;
        last_target_dwt_cnt_ = DWT->CYCCNT;
        feedback_dwt_ready_ = true;
        target_dwt_ready_ = true;
    }

    const DifferentialSwerveWheelKinematic::Matrix2x2&
    DifferentialSwerveWheelKinematic::GetControlMatrix() const {
        return control_matrix_;
    }

    const DifferentialSwerveWheelKinematic::Matrix2x2&
    DifferentialSwerveWheelKinematic::GetInverseMatrix() const {
        return inverse_matrix_;
    }

    bool DifferentialSwerveWheelKinematic::IsInvertible() const {
        return invertible_;
    }

    float DifferentialSwerveWheelKinematic::GetDriveAngleTarget() const {
        return target_drive_angle_;
    }

    float DifferentialSwerveWheelKinematic::Determinant(const Matrix2x2& matrix) {
        return matrix.a11 * matrix.a22 - matrix.a12 * matrix.a21;
    }

    DifferentialSwerveWheelKinematic::Matrix2x2 DifferentialSwerveWheelKinematic::Inverse(
        const Matrix2x2& matrix) {
        const float det = Determinant(matrix);
        if (det > -kSingularEpsilon && det < kSingularEpsilon) {
            return {0.0f, 0.0f, 0.0f, 0.0f};
        }

        const float inv_det = 1.0f / det;
        return {
            matrix.a22 * inv_det,
            -matrix.a12 * inv_det,
            -matrix.a21 * inv_det,
            matrix.a11 * inv_det,
        };
    }

}  // namespace control
