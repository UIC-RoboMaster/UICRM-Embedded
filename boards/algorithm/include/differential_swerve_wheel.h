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

#pragma once

#include "bsp_dwt.h"

namespace control {

    class DifferentialSwerveWheelKinematic {
      public:
        struct Vector2 {
            float x1;
            float x2;
        };

        struct Matrix2x2 {
            float a11;
            float a12;
            float a21;
            float a22;
        };

        struct WheelState {
            float yaw_angle;
            float drive_angle;
            float drive_speed;
        };

        struct MotorTarget {
            float motor1_angle;
            float motor2_angle;
        };

        explicit DifferentialSwerveWheelKinematic(
            const Matrix2x2& control_matrix = DefaultControlMatrix());

        // State order is always [yaw, drive].
        // x_dot = B * u, where:
        // yaw_dot   = a11 * motor1 + a12 * motor2
        // drive_dot = a21 * motor1 + a22 * motor2
        Vector2 InverseSolve(float yaw_rate, float drive_speed) const;
        void InverseSolve(float yaw_rate, float drive_speed, float& motor1_omega,
                          float& motor2_omega) const;

        // Solve wheel yaw rate and drive speed from motor angular velocities.
        Vector2 ForwardSolve(float motor1_omega, float motor2_omega) const;
        void ForwardSolve(float motor1_omega, float motor2_omega, float& yaw_rate,
                          float& drive_speed) const;

        // Estimate wheel yaw angle and drive speed from motor angles.
        WheelState Update(float motor1_theta, float motor2_theta);
        void Update(float motor1_theta, float motor2_theta, float& yaw_angle,
                    float& drive_speed);
        void Reset(float motor1_theta = 0.0f, float motor2_theta = 0.0f);

        // Solve motor target angles from target yaw angle and target drive speed.
        MotorTarget UpdateTarget(float yaw_angle_target, float drive_speed_target);
        void UpdateTarget(float yaw_angle_target, float drive_speed_target, float& motor1_angle,
                          float& motor2_angle);
        void ResetTarget(float yaw_angle_target = 0.0f, float drive_angle_target = 0.0f);

        const Matrix2x2& GetControlMatrix() const;
        const Matrix2x2& GetInverseMatrix() const;
        bool IsInvertible() const;
        float GetDriveAngleTarget() const;

        static constexpr Matrix2x2 DefaultControlMatrix() {
            return {
                5.6f, 5.6f,
                8.4f, -8.4f,
            };
        }

      private:
        static float Determinant(const Matrix2x2& matrix);
        static Matrix2x2 Inverse(const Matrix2x2& matrix);

      private:
        Matrix2x2 control_matrix_;
        Matrix2x2 inverse_matrix_;
        bool invertible_;
        WheelState state_;
        float target_drive_angle_;
        uint32_t last_feedback_dwt_cnt_;
        uint32_t last_target_dwt_cnt_;
        bool feedback_dwt_ready_;
        bool target_dwt_ready_;
    };

    // Backward-compatible alias for the previous misspelled name.
    using DifferentialSwerveWheelKinemetic = DifferentialSwerveWheelKinematic;

}  // namespace control
