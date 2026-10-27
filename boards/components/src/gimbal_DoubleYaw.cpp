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

#include "gimbal_DoubleYaw.h"

#include "DjiMotorBase.h"
#include "utils.h"

namespace control {

    Gimbal_DoubleYaw::Gimbal_DoubleYaw(gimbal_DoubleYaw_t gimbal) {
        pitch_motor_ = gimbal.pitch_motor;
        upper_yaw_motor_ = gimbal.upper_yaw_motor;
        lower_yaw_motor_ = gimbal.lower_yaw_motor;
        data_ = gimbal.data;

        // 上yaw(小yaw) 是主控轴，upper_yaw_angle_ 保存的是"目标朝向"（电机域），
        // 云台朝向以它为准；下yaw(大yaw)的目标在 Update* 里按协调结果计算
        pitch_target_ = 0.0f;
        upper_yaw_angle_ = data_.upper_yaw_offset_;
        lower_yaw_angle_ = data_.lower_yaw_offset_;

    }

    Gimbal_DoubleYaw::~Gimbal_DoubleYaw() {
    }

    gimbal_DoubleYaw_data_t* Gimbal_DoubleYaw::GetData() {
        return &data_;
    }

    void Gimbal_DoubleYaw::Recenter() {
        TargetAbs(0.0f, 0.0f);
        const float pitch_encoder = pitch_motor_->GetOutputShaftCumulatedTheta();
        pitch_motor_->SetTarget(
            pitch_encoder + wrapc<float>(data_.pitch_offset_ - pitch_encoder, -PI, PI), true);

        // 初始化要求两个关节各自归零，仅让关节角之和归零不能保证到达 offset。
        // offset 和反馈同属编码器域，无需再应用关节方向；选择最近的等价零点。
        const float upper_encoder = upper_yaw_motor_->GetOutputShaftCumulatedTheta();
        const float lower_encoder = lower_yaw_motor_->GetOutputShaftCumulatedTheta();
        upper_yaw_motor_->SetTarget(
            upper_encoder + wrapc<float>(data_.upper_yaw_offset_ - upper_encoder, -PI, PI));
        lower_yaw_angle_ =
            lower_encoder + wrapc<float>(data_.lower_yaw_offset_ - lower_encoder, -PI, PI);
        lower_yaw_motor_->SetTarget(lower_yaw_angle_);
    }

    void Gimbal_DoubleYaw::UpdateEncoder() {
        // 无 IMU 时，pitch 目标解释为相对标定位置的关节角。
        const float encoder = pitch_motor_->GetOutputShaftCumulatedTheta();
        const float joint = wrapc<float>(encoder - data_.pitch_offset_, -PI, PI);
        const float joint_target = clip<float>(pitch_target_, -data_.pitch_max_, data_.pitch_max_);
        pitch_motor_->SetTarget(encoder + joint_target - joint, true);

        // 没有IMU时，当前朝向取两个yaw电机角度之和
        float current_yaw = getUpperYawByMotor() + getLowerYawByMotor();
        // 目标朝向 = upper_yaw_angle_ - upper_yaw_offset_
        float target_yaw = upper_yaw_angle_ - data_.upper_yaw_offset_;
        CoordinateYaw(wrapc<float>(target_yaw - current_yaw, -PI, PI));
    }

    void Gimbal_DoubleYaw::UpdateIMU(float imu_pitch_angle, float imu_yaw_angle) {
        // IMU 随 pitch 转动，调用方将其正方向统一为编码器增大的方向。
        // 姿态目标以水平为零；只有编码器反馈需要消除整圈偏置。
        const float encoder = pitch_motor_->GetOutputShaftCumulatedTheta();
        const float joint = wrapc<float>(encoder - data_.pitch_offset_, -PI, PI);
        const float pitch_error = pitch_target_ - imu_pitch_angle;
        const float joint_target = clip<float>(
            joint + pitch_error, -data_.pitch_max_, data_.pitch_max_);
        float delta = joint_target - joint;

        // 已越界时仍允许拉回，不能用死区把关节保持在限位外。
        const bool within_limits = joint >= -data_.pitch_max_ && joint <= data_.pitch_max_;
        if (within_limits && abs(delta) < data_.pitch_deadband)
            delta = 0.0f;

        pitch_motor_->SetTarget(encoder + delta, true);
        // 机械饱和不改写 pitch_target_ 或 offset，可达后继续追踪原姿态目标。

        // ===== 大小yaw =====
        // IMU 装在云台(上yaw输出)上，imu_yaw 就是云台相对地面的朝向；
        // 目标朝向 = upper_yaw_angle_ - upper_yaw_offset_。
        // imu_yaw 与目标只差若干个 2π，所以不需要先折算到 [0, 2π]
        float yaw_diff = wrapc<float>(upper_yaw_angle_ - data_.upper_yaw_offset_ - imu_yaw_angle, -PI, PI);
        CoordinateYaw(yaw_diff);
    }

    void Gimbal_DoubleYaw::CoordinateYaw(float yaw_diff) {
        // 大小 yaw 轴的电机旋转方向
        const float sign_upper = data_.upper_yaw_inverted ? -1.0f : 1.0f;
        const float sign_lower = data_.lower_yaw_inverted ? -1.0f : 1.0f;

        const float u = getUpperYawByMotor();
        const float l = getLowerYawByMotor();
        const float upper_encoder = upper_yaw_motor_->GetOutputShaftCumulatedTheta();
        const float lower_encoder = lower_yaw_motor_->GetOutputShaftCumulatedTheta();

        // 下yaw同周期响应完整接手需求，不受上yaw朝向死区影响。
        float lower_delta = wrapc<float>(u + yaw_diff, -PI, PI);

        // 上yaw快速瞄准，只补偿下yaw已经实际完成的运动，不提前让角。
        if (abs(yaw_diff) < data_.upper_yaw_deadband)
            yaw_diff = 0.0f;
        float upper_target = u + yaw_diff;
        if (!data_.upper_yaw_circle_)
            upper_target = clip<float>(upper_target, -data_.upper_yaw_max_, data_.upper_yaw_max_);
        upper_yaw_motor_->SetTarget(upper_encoder + sign_upper * (upper_target - u));

        if (!data_.lower_yaw_circle_) {
            const float lower_target = clip<float>(l + lower_delta, -data_.lower_yaw_max_, data_.lower_yaw_max_);
            lower_delta = lower_target - l;
        }
        // 死区作用于完整接手误差；每周期明确下发保持/跟随目标。
        if (abs(lower_delta) < data_.lower_yaw_deadband)
            lower_delta = 0.0f;
        lower_yaw_angle_ = lower_encoder + sign_lower * lower_delta;
        lower_yaw_motor_->SetTarget(lower_yaw_angle_);
    }

    void Gimbal_DoubleYaw::TargetAbs(float abs_pitch, float abs_yaw) {
        if (data_.pitch_inverted)
            abs_pitch = -abs_pitch;
        pitch_target_ = clip<float>(abs_pitch, -data_.pitch_max_, data_.pitch_max_);

        // 目标朝向不受 upper_yaw_max_ 限幅：±90° 是上yaw关节相对下yaw的行程，
        // 只要大小yaw协调，云台相对车身/地面可以指向任意角度
        upper_yaw_angle_ = wrapc<float>(abs_yaw + data_.upper_yaw_offset_, 0, 2 * PI);
    }

    void Gimbal_DoubleYaw::TargetRel(float rel_pitch, float rel_yaw) {
        if (data_.pitch_inverted)
            rel_pitch = -rel_pitch;
        // 保存饱和后的目标，持续同向输入不会绕圈，反向输入立即生效。
        pitch_target_ = clip<float>(pitch_target_ + rel_pitch, -data_.pitch_max_, data_.pitch_max_);
        // 相对量叠加在"目标朝向"上，具体怎么分给两个yaw由 Update* 里的协调逻辑决定
        upper_yaw_angle_ = wrapc<float>(upper_yaw_angle_ + rel_yaw, 0, 2 * PI);
    }

    void Gimbal_DoubleYaw::SetPitchOffset(float encoder_at_level) {
        data_.pitch_offset_ = wrapc<float>(encoder_at_level, 0, 2 * PI);
    }

    void Gimbal_DoubleYaw::UpdateOffset(float pitch_offset_delta, float upper_yaw_offset, float lower_yaw_offset) {
        // pitch 目标独立于 offset；yaw 目标仍包含 offset，更新时保持其相对值。
        float upper_yaw_relative = upper_yaw_angle_ - data_.upper_yaw_offset_;

        SetPitchOffset(data_.pitch_offset_ + pitch_offset_delta);
        data_.upper_yaw_offset_ = wrap<float>(upper_yaw_offset + data_.upper_yaw_offset_, 0, 2 * PI);
        data_.lower_yaw_offset_ = wrap<float>(lower_yaw_offset + data_.lower_yaw_offset_, 0, 2 * PI);

        upper_yaw_angle_ = wrapc<float>(upper_yaw_relative + data_.upper_yaw_offset_, 0, 2 * PI);
    }

    float Gimbal_DoubleYaw::getPitchTarget() const {
        return pitch_target_;
    }

    float Gimbal_DoubleYaw::getUpperYawTarget() const {
        return upper_yaw_angle_;
    }

    float Gimbal_DoubleYaw::getLowerYawTarget() const {
        return lower_yaw_angle_;
    }

    float Gimbal_DoubleYaw::getPitchByMotor() const {
        return wrapc<float>(pitch_motor_->GetOutputShaftCumulatedTheta() - data_.pitch_offset_, -PI, PI);
    }

    float Gimbal_DoubleYaw::getUpperYawByMotor() const {
        // 上yaw编码器测的是它相对下yaw(定子)的角度，减去标定中心就是关节角。
        // 机械行程为 ±90°，但累计反馈可能带有整圈偏置；先消除任意整圈偏置，
        // 再由 CoordinateYaw 对相对 offset 的关节目标限幅。
        float angle = upper_yaw_motor_->GetOutputShaftCumulatedTheta() - data_.upper_yaw_offset_;
        if (data_.upper_yaw_inverted)
            angle = -angle;
        return wrapc<float>(angle, -PI, PI);
    }

    float Gimbal_DoubleYaw::getLowerYawByMotor() const {
        float angle = lower_yaw_motor_->GetOutputShaftCumulatedTheta() - data_.lower_yaw_offset_;
        if (data_.lower_yaw_inverted)
            angle = -angle;
        // 有限行程以最近的机械中心为零，去掉累计反馈中的整圈偏置；连续旋转保留圈数。
        if (!data_.lower_yaw_circle_)
            angle = wrapc<float>(angle, -PI, PI);
        return angle;
    }
}  // namespace control
