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

#include "FlyWheelMotor.h"

#include "utils.h"

namespace driver {

FlyWheelMotor::FlyWheelMotor(flywheel_t data) {
    motor_ = data.motor;
    max_speed_ = data.max_speed;
    target_speed_ = 0;
    is_inverted_ = data.is_inverted;
    omega_pid_ = control::PIDController(data.omega_pid_param);
}

void FlyWheelMotor::SetSpeed(float speed) {
    if (is_inverted_) {
        speed = -speed;
    }
    speed = clip<float>(speed, -max_speed_, max_speed_);
    target_speed_ = speed;
}

void FlyWheelMotor::CalcOutput() {
    motor_->SetOutput(
        omega_pid_.ComputeConstrainedOutput(motor_->GetOmegaDelta(target_speed_)));
}

float FlyWheelMotor::GetTarget() const {
    return is_inverted_ ? -target_speed_ : target_speed_;
}

void FlyWheelMotor::PrintData() const {
    print("Fly-target: %2.5f ", target_speed_);
    motor_->PrintData();
}

void FlyWheelMotor::UpdateData(const uint8_t data[]) {
    motor_->UpdateData(data);
}

float FlyWheelMotor::GetTheta() const {
    return motor_->GetTheta();
}

float FlyWheelMotor::GetThetaDelta(const float target) const {
    return motor_->GetThetaDelta(target);
}

float FlyWheelMotor::GetOmega() const {
    return motor_->GetOmega();
}

float FlyWheelMotor::GetOmegaDelta(const float target) const {
    return motor_->GetOmegaDelta(target);
}

}  // namespace driver
