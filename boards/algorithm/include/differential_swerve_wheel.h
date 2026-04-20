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
#include "pid.h"
#include "utils.h"

namespace control {

    class DifferentialSwerveWheel {
      public:
        struct Matrix2x2 {
            float a11;
            float a12;
            float a21;
            float a22;
        };

        struct MotorTarget {
            float motor1_theta;
            float motor2_theta;
            float motor1_omega;
            float motor2_omega;
        };

        struct State {
            float yaw_angle;
            float yaw_rate;
            float drive_angle;
            float drive_speed;
            float motor1_theta;
            float motor2_theta;
            float motor1_omega;
            float motor2_omega;
        };

        struct Config {
            // x_dot = B * u, u = [motor1_omega, motor2_omega]^T
            // The default assumes a standard differential-swerve mapping:
            // yaw depends on motor speed difference and drive depends on motor speed sum.
            Matrix2x2 kinematics = {
                .a11 = 8.4f,
                .a12 = -8.4f,
                .a21 = 5.6f,
                .a22 = 5.6f,
            };

            ConstrainedPID::PID_Init_t yaw_pid_init = {
                .kp = 10.0f,
                .ki = 0.0f,
                .kd = 0.1f,
                .max_out = 12.0f,
                .max_iout = 0.0f,
                .deadband = 0.0f,
                .A = 0.0f,
                .B = 0.0f,
                .output_filtering_coefficient = 0.0f,
                .derivative_filtering_coefficient = 0.0f,
                .mode = ConstrainedPID::OutputFilter,
            };

            float max_yaw_rate = 6.0f * PI;
            float max_drive_speed = 30.0f * PI;
            float max_motor_speed = 30.0f * PI;
            float min_dt = 1e-4f;
            float max_dt = 0.05f;
            float singular_epsilon = 1e-5f;
        };

        explicit DifferentialSwerveWheel(const Config& config = Config());

        void Reset(float motor1_theta = 0.0f, float motor2_theta = 0.0f);
        void UpdateFeedback(float motor1_theta, float motor2_theta);
        MotorTarget Update(float motor1_theta, float motor2_theta);

        void SetTarget(float yaw_relative, float drive_speed);
        void SetYawTarget(float yaw_relative);
        void SetDriveSpeed(float drive_speed);

        void SetKinematics(const Matrix2x2& kinematics);
        void ReInitYawPID(const ConstrainedPID::PID_Init_t& pid_init);
        void SetMaxYawRate(float max_yaw_rate);
        void SetMaxDriveSpeed(float max_drive_speed);
        void SetMaxMotorSpeed(float max_motor_speed);

        const MotorTarget& GetMotorTarget() const;
        const State& GetState() const;
        float GetYawTarget() const;
        float GetDriveSpeedTarget() const;
        bool IsKinematicsInvertible() const;

      private:
        void UpdateStateFromFeedback(float motor1_theta, float motor2_theta, float dt);
        void SolveMotorOmega(float yaw_rate, float drive_speed, float& motor1_omega,
                             float& motor2_omega) const;
        float GetDeltaT();
        void RefreshInverse();

      private:
        Config config_;
        ConstrainedPID yaw_pid_;

        Matrix2x2 inverse_ = {};
        bool invertible_ = false;

        State state_ = {};
        MotorTarget motor_target_ = {};

        float yaw_target_ = 0.0f;
        float drive_speed_target_ = 0.0f;
        float yaw_zero_ = 0.0f;
        float drive_zero_ = 0.0f;

        float last_motor1_theta_ = 0.0f;
        float last_motor2_theta_ = 0.0f;
        bool has_feedback_ = false;

        uint32_t last_dwt_cnt_ = 0;
        bool dwt_ready_ = false;
    };

}  // namespace control
