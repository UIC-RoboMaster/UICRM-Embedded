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

#include "gimbal.h"

#include "MotorCanBase.h"
#include "utils.h"

namespace control {

    Gimbal::Gimbal(gimbal_t gimbal)
         {
        // acquired from user
        pitch_motor_ = gimbal.pitch_motor;
        yaw_motor_ = gimbal.yaw_motor;
        data_ = gimbal.data;

        target_pitch_ = data_.pitch_offset_;
        target_yaw_ = data_.yaw_offset_;

        pitch_lower_limit_ = wrap<float>(data_.pitch_offset_ - data_.pitch_max_, 0, 2 * PI);
        pitch_upper_limit_ = wrap<float>(data_.pitch_offset_ + data_.pitch_max_, 0, 2 * PI);
        yaw_lower_limit_ = wrap<float>(data_.yaw_offset_ - data_.yaw_max_, 0, 2 * PI);
        yaw_upper_limit_ = wrap<float>(data_.yaw_offset_ + data_.yaw_max_, 0, 2 * PI);
    }

    Gimbal::~Gimbal() {
    }

    gimbal_data_t* Gimbal::GetData() {
        return &data_;
    }



    void Gimbal::Update() {

        pitch_motor_->SetTarget(target_pitch_);

//        float pt_diff = pitch_motor_->GetThetaDelta(target_pitch_);
//        pt_diff = wrap<float>(pt_diff, -PI, PI);
//
//        if (abs(pt_diff) < data_.pitch_eposition) {
//            pt_diff = 0;
//        }
//        float pt_out = pitch_theta_pid_->ComputeOutput(pt_diff);
//
//        float po_in = pitch_motor_->GetOmegaDelta(pt_out);
//        float po_out = pitch_omega_pid_->ComputeConstrainedOutput(po_in);

        yaw_motor_->SetTarget(target_yaw_);

//        float yt_diff = yaw_motor_->GetThetaDelta(target_yaw_);
//        yt_diff = wrap<float>(yt_diff, -PI, PI);
//
//        if (abs(yt_diff) < data_.yaw_eposition) {
//            yt_diff = 0;
//        }
//
//        float yt_out = yaw_theta_pid_->ComputeOutput(yt_diff);
//        float yo_in = yaw_motor_->GetOmegaDelta(yt_out);
//        float yo_out = yaw_omega_pid_->ComputeConstrainedOutput(yo_in);
//
//        pitch_motor_->SetOutput(po_out);
//        yaw_motor_->SetOutput(yo_out);
    }

    void Gimbal::UpdateIMU(float pitch, float yaw) {
        // 有：目标角度，当前电机角度，当前IMU角度
        // 要做的：计算电机输出，使得？
        float pitch_diff = target_pitch_ - data_.pitch_offset_ - pitch;
        float actual_pitch = pitch_motor_->GetTheta();
        float new_pitch_diff = wrapping_clip<float>(
            pitch_diff + actual_pitch, pitch_lower_limit_, pitch_upper_limit_, 0, 2 * PI);
        new_pitch_diff = new_pitch_diff - actual_pitch;
        if (pitch_diff != new_pitch_diff) {
            target_pitch_ = wrap<float>(target_pitch_ + new_pitch_diff - pitch_diff, 0, 2 * PI);
        }
        pitch_diff = wrap<float>(new_pitch_diff, -PI, PI);

        if (abs(pitch_diff) < data_.pitch_eposition) {
            pitch_diff = 0;
        }
        pitch_motor_->SetTarget(actual_pitch + pitch_diff);

//        float pt_out = pitch_theta_pid_->ComputeOutput(pitch_diff);
//        float po_in = pitch_motor_->GetOmegaDelta(pt_out);
//        float po_out = pitch_omega_pid_->ComputeConstrainedOutput(po_in);


        float yaw_diff = target_yaw_ - data_.yaw_offset_ - yaw;
        float actual_yaw = yaw_motor_->GetTheta();
        if (!data_.yaw_circle_) {

            float new_yaw_diff = wrapping_clip<float>(yaw_diff + actual_yaw, yaw_lower_limit_,
                                                      yaw_upper_limit_, 0, 2 * PI);
            new_yaw_diff = new_yaw_diff - actual_yaw;
            if (yaw_diff != new_yaw_diff) {
                target_yaw_ = wrap<float>(target_yaw_ + new_yaw_diff - yaw_diff, 0, 2 * PI);
            }
            yaw_diff = wrap<float>(new_yaw_diff, -PI, PI);
        } else {
            yaw_diff = wrap<float>(yaw_diff, -PI, PI);
        }

        if (abs(yaw_diff) < data_.yaw_eposition) {
            yaw_diff = 0;
        }

        yaw_motor_->SetTarget(actual_yaw + yaw_diff);

//        float yt_out = yaw_theta_pid_->ComputeOutput(yaw_diff);
//        float yo_in = yaw_motor_->GetOmegaDelta(yt_out);
//        float yo_out = yaw_omega_pid_->ComputeConstrainedOutput(yo_in);
//        pitch_motor_->SetOutput(po_out);
//        yaw_motor_->SetOutput(yo_out);
    }

    void Gimbal::SetAbsTarget(float abs_pitch, float abs_yaw) {
        if (data_.pitch_inverted)
            abs_pitch = -abs_pitch;
        if (data_.yaw_inverted)
            abs_yaw = -abs_yaw;
        float clipped_pitch = clip<float>(abs_pitch, -data_.pitch_max_, data_.pitch_max_);
        float clipped_yaw = clip<float>(abs_yaw, -data_.yaw_max_, data_.yaw_max_);
        target_pitch_ = wrapping_clip<float>(clipped_pitch + data_.pitch_offset_, pitch_lower_limit_,
                                            pitch_upper_limit_, 0, 2 * PI);
        if(data_.yaw_circle_){
            target_yaw_ = wrap<float>(clipped_yaw + data_.yaw_offset_, 0, 2 * PI);
        }
        else{
            target_yaw_ = wrapping_clip<float>(clipped_yaw + data_.yaw_offset_, yaw_lower_limit_,
                                              yaw_upper_limit_, 0, 2 * PI);
        }

    }



    void Gimbal::SetRelTarget(float rel_pitch, float rel_yaw) {
        if (data_.pitch_inverted)
            rel_pitch = -rel_pitch;
        if (data_.yaw_inverted)
            rel_yaw = -rel_yaw;
        target_pitch_ = wrap<float>(target_pitch_ + rel_pitch, 0, 2 * PI);
        target_yaw_ = wrap<float>(target_yaw_ + rel_yaw, 0, 2 * PI);
    }

    void Gimbal::UpdateOffset(float pitch_offset, float yaw_offset) {
        data_.pitch_offset_ = wrap<float>(pitch_offset + data_.pitch_offset_, 0, 2 * PI);
        data_.yaw_offset_ = wrap<float>(yaw_offset + data_.yaw_offset_, 0, 2 * PI);
    }

}  // namespace control
