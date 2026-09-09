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

#include "ServoMotor.h"

#include "arm_math.h"
#include "bsp_os.h"
#include "utils.h"

namespace driver {

    static void servomotor_callback(const uint8_t data[], void* args) {
        ServoMotor* servo = reinterpret_cast<ServoMotor*>(args);
        servo->UpdateData(data);
    }

    ServoMotor::ServoMotor(servo_t data, float align_angle, float proximity_in, float proximity_out) {
        motor_ = data.motor;
        max_speed_ = data.transmission_ratio * data.max_speed;
        max_acceleration_ = data.transmission_ratio * data.max_acceleration;
        transmission_ratio_ = data.transmission_ratio;
        proximity_in_ = proximity_in;
        proximity_out_ = proximity_out;

        hold_ = true;
        target_angle_ = 0;
        align_angle_ = align_angle;
        motor_angle_ = 0;
        offset_angle_ = 0;
        servo_angle_ = 0;
        cumulated_angle_ = 0;
        inner_wrap_detector_ = new FloatEdgeDetector(0, PI);
        outer_wrap_detector_ = new FloatEdgeDetector(0, PI);
        hold_detector_ = new BoolEdgeDetector(false);

        omega_pid_.Reinit(data.omega_pid_param, data.max_iout, data.max_out);
        hold_pid_.Reinit(data.hold_pid_param, data.hold_max_iout, data.hold_max_out);

        data.motor->can_->RegisterRxCallback(data.motor->rx_id_, servomotor_callback, this);

        jam_callback_ = nullptr;
        detect_head_ = -1;
        detect_period_ = -1;
        detect_total_ = 0;
        detect_buf_ = nullptr;
    }

    servo_status_t ServoMotor::SetTarget(const float target, bool override) {
        if (!hold_ && !override)
            return INPUT_REJECT;
        servo_status_t dir = target < target_angle_ ? TURNING_ANTICLOCKWISE : TURNING_CLOCKWISE;
        target_angle_ = target;
        return dir;
    }

    void ServoMotor::SetMaxSpeed(const float max_speed) {
        if (max_speed > 0)
            max_speed_ = transmission_ratio_ * max_speed;
        else
            RM_EXPECT_TRUE(false, "Max speed should be positive");
    }

    void ServoMotor::SetMaxAcceleration(const float max_acceleration) {
        if (max_acceleration > 0)
            max_acceleration_ = transmission_ratio_ * max_acceleration;
        else
            RM_EXPECT_TRUE(false, "Max acceleration should be positive");
    }

    void ServoMotor::CalcOutput() {
        hold_detector_->input(hold_);
        if (hold_detector_->edge()) {
            omega_pid_.Reset();
            hold_pid_.Reset();
        }

        if (hold_detector_->negEdge())
            start_time_ = bsp::GetHighresTickMicroSec();

        int16_t command;
        float target_diff = (target_angle_ - servo_angle_ - cumulated_angle_) * transmission_ratio_;
        uint64_t current_time = bsp::GetHighresTickMicroSec();
        if (!hold_) {
            float speed_max_start = (current_time - start_time_) / 10e6 * max_acceleration_ * transmission_ratio_;
            float speed_max_target = sqrt(2 * max_acceleration_ * abs(target_diff));
            float current_speed = speed_max_start > speed_max_target ? speed_max_target : speed_max_start;
            current_speed = clip<float>(current_speed, 0, max_speed_);
            command =
                omega_pid_.ComputeConstrainedOutput(motor_->GetOmegaDelta(sign<float>(target_diff, 0) * current_speed));
        } else {
            command = hold_pid_.ComputeConstrainedOutput(motor_->GetOmegaDelta(target_diff * 50));
        }
        motor_->SetOutput(command);

        if (detect_buf_ != nullptr) {
            detect_total_ += command - detect_buf_[detect_head_];
            detect_buf_[detect_head_] = command;
            detect_head_ = detect_head_ + 1 < detect_period_ ? detect_head_ + 1 : 0;

            jam_detector_->input(abs(detect_total_) >= jam_threshold_);
            if (jam_detector_->posEdge()) {
                servo_jam_t data;
                data.speed = max_speed_ / transmission_ratio_;
                jam_callback_(this, data);
            }
        }
    }

    void ServoMotor::Hold(bool override) {
        if (!Holding()) {
            SetTarget(GetTheta(), override);
        }
    }

    bool ServoMotor::Holding() const {
        return hold_;
    }

    float ServoMotor::GetTarget() const {
        return target_angle_;
    }

    void ServoMotor::RegisterJamCallback(jam_callback_t callback, float effort_threshold, uint8_t detect_period) {
        constexpr int maximum_command = 32768;
        RM_ASSERT_TRUE(effort_threshold > 0 && effort_threshold <= 1, "Effort threshold should between 0 and 1");
        jam_callback_ = callback;

        detect_head_ = 0;
        detect_period_ = detect_period;
        detect_total_ = 0;
        if (detect_buf_ != nullptr)
            delete detect_buf_;
        detect_buf_ = new int16_t[detect_period];
        memset(detect_buf_, 0, detect_period);

        jam_threshold_ = maximum_command * effort_threshold * detect_period;
        jam_detector_ = new BoolEdgeDetector(false);
    }

    void ServoMotor::PrintData() const {
        print("Svo-align: % 10.6f ", align_angle_);
        print("Svo-theta: % 10.6f ", GetTheta());
        print("Svo-omega: % 10.6f ", GetOmega());
        print("Svo-target: % 10.6f ", target_angle_);
        if (hold_)
            print("Svo-status: holding ");
        else
            print("Svo-status: moving  ");
        motor_->PrintData();
    }

    float ServoMotor::GetTheta() const {
        return servo_angle_ + cumulated_angle_;
    }

    float ServoMotor::GetThetaDelta(const float target) const {
        return target - GetTheta();
    }

    float ServoMotor::GetOmega() const {
        return motor_->GetOmega() / transmission_ratio_;
    }

    float ServoMotor::GetOmegaDelta(const float target) const {
        return target - motor_->GetOmega() / transmission_ratio_;
    }

    void ServoMotor::UpdateData(const uint8_t data[]) {
        motor_->UpdateData(data);

        if (align_angle_ < 0)
            align_angle_ = motor_->GetTheta();

        motor_angle_ = motor_->GetTheta() - align_angle_;
        inner_wrap_detector_->input(motor_angle_);
        if (inner_wrap_detector_->negEdge())
            offset_angle_ = wrap<float>(offset_angle_ + 2 * PI / transmission_ratio_, 0, 2 * PI);
        else if (inner_wrap_detector_->posEdge())
            offset_angle_ = wrap<float>(offset_angle_ - 2 * PI / transmission_ratio_, 0, 2 * PI);

        servo_angle_ = wrap<float>(offset_angle_ + motor_angle_ / transmission_ratio_, 0, 2 * PI);
        outer_wrap_detector_->input(servo_angle_);
        if (outer_wrap_detector_->negEdge())
            cumulated_angle_ += 2 * PI;
        else if (outer_wrap_detector_->posEdge())
            cumulated_angle_ -= 2 * PI;

        float diff = abs(GetThetaDelta(target_angle_));
        if (!hold_ && diff < proximity_in_)
            hold_ = true;
        if (hold_ && diff > proximity_out_)
            hold_ = false;
    }

}  // namespace driver
