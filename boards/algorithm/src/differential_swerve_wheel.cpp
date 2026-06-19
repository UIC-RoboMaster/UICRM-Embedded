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

#include "arm_math.h"
#include "utils.h"

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
          state_({0.0f, 0.0f, 0.0f, 0.0f}),
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

    inline float WrapToPi(float angle)
    {
        while (angle > PI)
        {
            angle -= 2.0f * PI;
        }

        while (angle <= -PI)
        {
            angle += 2.0f * PI;
        }

        return angle;
    }

    DifferentialSwerveWheelKinematic::WheelState
    DifferentialSwerveWheelKinematic::Update(
        float motor1_theta,
        float motor2_theta,
        float m1_trans_ratio,
        float m2_trans_ratio) {

        // 1. 转换到输入端角度
        const float input1_theta = motor1_theta / m1_trans_ratio;
        const float input2_theta = motor2_theta / m2_trans_ratio;

        // 2. 正解计算当前的绝对角度
        const float yaw_angle_raw =
            control_matrix_.a11 * input1_theta +
            control_matrix_.a12 * input2_theta;

        const float drive_angle =
            control_matrix_.a21 * input1_theta +
            control_matrix_.a22 * input2_theta;

        float drive_speed = 0.0f;

        // 3. 计算时间步长 dt
        if (feedback_dwt_ready_) {
            float dt = DWT_GetDeltaT(&last_feedback_dwt_cnt_);
            if (dt < kMinDt) {
                dt = kMinDt;
            }

            // 4. 安全的速度计算：处理潜在的角度回绕
            // 如果 drive_angle 是连续累加且不溢出的 float，可直接减；
            // 如果 drive_angle 存在范围限制（如 0~2pi 或因编码器溢出断点），需计算最短步长：
            float delta_drive = drive_angle - state_.drive_angle;

            // 关键安全保障：假设单帧内驱动轮转动不超过半圈（π rad），防止由于回绕导致的速度暴走
            // 如果你的 drive_angle 是纯连续累加且永不溢出的 double/float，可以注释掉下面这行
            // delta_drive = WrapToPi(delta_drive);

            drive_speed = delta_drive / dt;
        } else {
            last_feedback_dwt_cnt_ = DWT->CYCCNT;
            feedback_dwt_ready_ = true;
        }

        // 5. 更新状态
        state_.yaw_angle_raw = yaw_angle_raw;
        state_.yaw_angle = WrapToPi(yaw_angle_raw); // 转向角限制在 [-PI, PI]

        state_.drive_angle = drive_angle;
        state_.drive_speed = drive_speed;

        return state_;
    }

    void DifferentialSwerveWheelKinematic::Update(float motor1_theta, float motor2_theta,
                                                  float& yaw_angle, float& drive_speed,
                                                  float m1_trans_ratio, float m2_trans_ratio) {
        const WheelState state = Update(motor1_theta, motor2_theta, m1_trans_ratio, m2_trans_ratio);
        yaw_angle = state.yaw_angle_raw;
        drive_speed = state.drive_speed;
    }

    void DifferentialSwerveWheelKinematic::Reset(float motor1_theta, float motor2_theta) {
        state_.yaw_angle_raw = control_matrix_.a11 * motor1_theta + control_matrix_.a12 * motor2_theta;
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

        state_.yaw_angle_raw = yaw_angle_target;
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
