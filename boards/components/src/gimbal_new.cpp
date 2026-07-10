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

#include "gimbal_new.h"

#include "utils.h"

namespace control {

    Gimbal::Gimbal(gimbal_t gimbal) {
        // acquired from user
        pitch_motor_ = gimbal.pitch_motor;
        yaw_motor_ = gimbal.yaw_motor;
        data_ = gimbal.data;

        pitch_angle_ = data_.pitch_offset_;
        yaw_angle_ = data_.yaw_offset_;

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
        pitch_angle_ = wrapping_clip<float>(pitch_angle_, pitch_lower_limit_, pitch_upper_limit_, 0, 2 * PI);
        pitch_motor_->SetTarget(pitch_angle_);

        if (!data_.yaw_circle_) {
            yaw_angle_ = wrapping_clip<float>(yaw_angle_, yaw_lower_limit_, yaw_upper_limit_, 0, 2 * PI);
        }
        yaw_motor_->SetTarget(yaw_angle_);
    }

    // update with imu
    void Gimbal::UpdateIMU(float imu_pitch_angle, float imu_yaw_angle) {
        // ===== Pitch 轴 =====
        // 当前角度和目标角度的差值
        float pitch_diff = pitch_angle_ - data_.pitch_offset_ - imu_pitch_angle;
        // 当前电机实际角度
        float current_pitch_angle = pitch_motor_->GetOutputShaftTheta();
        // 目标角度
        float target_pitch_diff =
            wrapping_clip<float>(pitch_diff + current_pitch_angle, pitch_lower_limit_, pitch_upper_limit_, 0, 2 * PI);
        target_pitch_diff = target_pitch_diff - current_pitch_angle;

        // 如果超过限位
        if (pitch_diff != target_pitch_diff) {
            pitch_angle_ = wrap<float>(pitch_angle_ + target_pitch_diff - pitch_diff, 0, 2 * PI);
        }
        // 得到最终的差值
        float final_pitch_diff = wrap<float>(target_pitch_diff, -PI, PI);

        // 死区，不动
        if (abs(final_pitch_diff) < data_.pitch_eposition) {
            final_pitch_diff = 0;
        }
        pitch_motor_->SetTarget(current_pitch_angle + final_pitch_diff);


        // ===== Yaw 轴 =====
        float yaw_diff = yaw_angle_ - data_.yaw_offset_ - imu_yaw_angle;
        // 当前 yaw 轴电机实际角度
        float current_yaw_angle = yaw_motor_->GetOutputShaftTheta();
        float final_yaw_diff;
        if (!data_.yaw_circle_) {
            float target_yaw_diff =
                wrapping_clip<float>(yaw_diff + current_yaw_angle, yaw_lower_limit_, yaw_upper_limit_, 0, 2 * PI);
            target_yaw_diff = target_yaw_diff - current_yaw_angle;
            // 如果超过限位
            if (yaw_diff != target_yaw_diff) {
                yaw_angle_ = wrap<float>(yaw_angle_ + target_yaw_diff - yaw_diff, 0, 2 * PI);
            }

            final_yaw_diff = wrap<float>(target_yaw_diff, -PI, PI);
        } else {
            final_yaw_diff = wrap<float>(yaw_diff, -PI, PI);
        }

        // 死区处理
        if (abs(final_yaw_diff) < data_.yaw_eposition) {
            final_yaw_diff = 0;
        }

        yaw_motor_->SetTarget(current_yaw_angle + final_yaw_diff);
    }

    void Gimbal::TargetAbs(float abs_pitch, float abs_yaw) {
        if (data_.pitch_inverted)
            abs_pitch = -abs_pitch;
        if (data_.yaw_inverted)
            abs_yaw = -abs_yaw;
        float clipped_pitch = clip<float>(abs_pitch, -data_.pitch_max_, data_.pitch_max_);
        float clipped_yaw = clip<float>(abs_yaw, -data_.yaw_max_, data_.yaw_max_);
        pitch_angle_ = wrapping_clip<
            float>(clipped_pitch + data_.pitch_offset_, pitch_lower_limit_, pitch_upper_limit_, 0, 2 * PI);
        if (data_.yaw_circle_) {
            yaw_angle_ = wrap<float>(clipped_yaw + data_.yaw_offset_, 0, 2 * PI);
        } else {
            yaw_angle_ =
                wrapping_clip<float>(clipped_yaw + data_.yaw_offset_, yaw_lower_limit_, yaw_upper_limit_, 0, 2 * PI);
        }
    }

    void Gimbal::TargetRel(float rel_pitch, float rel_yaw) {
        if (data_.pitch_inverted)
            rel_pitch = -rel_pitch;
        if (data_.yaw_inverted)
            rel_yaw = -rel_yaw;
        // TODO limit？
        pitch_angle_ = wrap<float>(pitch_angle_ + rel_pitch, 0, 2 * PI);
        // TODO limit？
        yaw_angle_ = wrap<float>(yaw_angle_ + rel_yaw, 0, 2 * PI);
    }

    void Gimbal::UpdateOffset(float pitch_offset, float yaw_offset) {
        data_.pitch_offset_ = wrap<float>(pitch_offset + data_.pitch_offset_, 0, 2 * PI);
        data_.yaw_offset_ = wrap<float>(yaw_offset + data_.yaw_offset_, 0, 2 * PI);
        // TODO 更新？

    }

    void Gimbal::TargetReal(float new_pitch, float new_yaw) {
        if (data_.pitch_inverted)
            new_pitch = -new_pitch;
        if (data_.yaw_inverted)
            new_yaw = -new_yaw;
        pitch_angle_ = wrap<float>(pitch_angle_ + new_pitch, 0, 2 * PI);
        yaw_angle_ = wrap<float>(yaw_angle_ + new_yaw, 0, 2 * PI);
    }

    float Gimbal::getPitchTarget() const {
        return pitch_angle_;
    }
    float Gimbal::getYawTarget() const {
        return yaw_angle_;
    }
    float Gimbal::getPitchByMotor() const {
        return pitch_motor_->GetOutputShaftTheta() - data_.pitch_offset_;
    }
    float Gimbal::getYawByMotor() const {
        return yaw_motor_->GetOutputShaftTheta() - data_.yaw_offset_;
    }
}  // namespace control
