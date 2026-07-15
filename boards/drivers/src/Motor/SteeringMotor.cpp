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

#include "SteeringMotor.h"

#include "arm_math.h"
#include "utils.h"

namespace driver {

    SteeringMotor::SteeringMotor(steering_t data) {
        servo_t servo_data;
        servo_data.motor = data.motor;
        servo_data.max_speed = data.max_speed;
        servo_data.max_acceleration = data.max_acceleration;
        servo_data.transmission_ratio = data.transmission_ratio;
        servo_data.omega_pid_param = data.omega_pid_param;
        servo_data.max_iout = data.max_iout;
        servo_data.max_out = data.max_out;
        servo_ = new ServoMotor(servo_data, data.offset_angle);

        test_speed_ = data.test_speed;
        align_detect_func = data.align_detect_func;
        calibrate_offset = data.calibrate_offset;
        align_angle_ = 0;
        align_detector = new BoolEdgeDetector(false);
        align_complete_ = false;
    }

    float SteeringMotor::GetRawTheta() const {
        return servo_->GetTheta();
    }

    void SteeringMotor::PrintData() const {
        print("Str-align: %10.5f ", align_angle_);
        servo_->PrintData();
    }

    void SteeringMotor::TurnRelative(float angle) {
        servo_->SetTarget(servo_->GetTarget() + angle, true);
    }

    void SteeringMotor::TurnAbsolute(float angle) {
        servo_->SetTarget(angle);
    }

    bool SteeringMotor::AlignUpdate() {
        if (align_complete_) {
            servo_->SetTarget(align_angle_, true);
            servo_->CalcOutput();
            return true;
        } else if (align_detect_func()) {
            float current_theta = servo_->motor_->GetTheta();
            float offset = wrap<float>(servo_->align_angle_ - current_theta, -PI, PI);
            float current = (current_theta + offset - servo_->align_angle_) / servo_->transmission_ratio_ +
                            servo_->offset_angle_ + servo_->cumulated_angle_;
            align_angle_ = current + calibrate_offset;
            align_complete_ = true;
            servo_->SetTarget(align_angle_, true);
            servo_->CalcOutput();
            return true;
        } else {
            servo_->motor_->SetOutput(servo_->omega_pid_.ComputeConstrainedOutput(
                servo_->motor_->GetOmegaDelta(test_speed_ * servo_->transmission_ratio_)
            ));
        }
        return false;
    }

    void SteeringMotor::Update() {
        servo_->CalcOutput();
    }

}  // namespace driver
