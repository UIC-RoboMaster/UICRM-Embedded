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

#include "dual_yaw_gimbal.h"

#include "DjiMotorBase.h"
#include "utils.h"

namespace control {

    Dual_Yaw_Gimbal::Dual_Yaw_Gimbal(dual_yaw_gimbal_t gimbal) {
        pitch_motor_ = gimbal.pitch_motor;
        upper_yaw_motor_ = gimbal.upper_yaw_motor;
        lower_yaw_motor_ = gimbal.lower_yaw_motor;
        data_ = gimbal.data;

        // 上yaw(小yaw) 是主控轴，upper_yaw_angle_ 保存的是"目标朝向"（电机域），
        // 云台朝向以它为准；下yaw(大yaw)的目标在 Update* 里按协调结果计算
        pitch_angle_ = data_.pitch_offset_;
        upper_yaw_angle_ = data_.upper_yaw_offset_;
        lower_yaw_angle_ = data_.lower_yaw_offset_;

        pitch_lower_limit_ = wrap<float>(data_.pitch_offset_ - data_.pitch_max_, 0, 2 * PI);
        pitch_upper_limit_ = wrap<float>(data_.pitch_offset_ + data_.pitch_max_, 0, 2 * PI);
    }

    Dual_Yaw_Gimbal::~Dual_Yaw_Gimbal() {
    }

    dual_yaw_gimbal_data_t* Dual_Yaw_Gimbal::GetData() {
        return &data_;
    }

    void Dual_Yaw_Gimbal::Recenter() {
        TargetAbs(0.0f, 0.0f);
        pitch_motor_->SetTarget(pitch_angle_);

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

    void Dual_Yaw_Gimbal::UpdateEncoder() {
        // Pitch 轴
        pitch_angle_ = wrapping_clip<float>(pitch_angle_, pitch_lower_limit_, pitch_upper_limit_, 0, 2 * PI);
        pitch_motor_->SetTarget(pitch_angle_);

        // 没有IMU时，当前朝向取两个yaw电机角度之和
        float current_yaw = getUpperYawByMotor() + getLowerYawByMotor();
        // 目标朝向 = upper_yaw_angle_ - upper_yaw_offset_
        float target_yaw = upper_yaw_angle_ - data_.upper_yaw_offset_;
        CoordinateYaw(wrapc<float>(target_yaw - current_yaw, -PI, PI));
    }

    void Dual_Yaw_Gimbal::UpdateIMU(float imu_pitch_angle, float imu_yaw_angle) {
        // IMU 输出范围为 [-π, π]，电机范围为 [0, 2π]，统一转到电机域
        imu_pitch_angle = wrap<float>(imu_pitch_angle, 0, 2 * PI);

        // Pitch 轴
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
        // -PI - PI 不是归化值域，而是计算最短半径
        float final_pitch_diff = wrap<float>(target_pitch_diff, -PI, PI);

        // 死区，不动
        if (abs(final_pitch_diff) < data_.pitch_deadband) {
            final_pitch_diff = 0;
        }

        pitch_motor_->SetTarget(current_pitch_angle + final_pitch_diff);

        // ===== 大小yaw =====
        // IMU 装在云台(上yaw输出)上，imu_yaw 就是云台相对地面的朝向；
        // 目标朝向 = upper_yaw_angle_ - upper_yaw_offset_。
        // imu_yaw 与目标只差若干个 2π，所以不需要先折算到 [0, 2π]
        float yaw_diff = wrapc<float>(upper_yaw_angle_ - data_.upper_yaw_offset_ - imu_yaw_angle, -PI, PI);
        CoordinateYaw(yaw_diff);
    }

    void Dual_Yaw_Gimbal::CoordinateYaw(float yaw_diff) {
        const float sign_upper = data_.upper_yaw_joint_inverted ? -1.0f : 1.0f;
        const float sign_lower = data_.lower_yaw_joint_inverted ? -1.0f : 1.0f;
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

    void Dual_Yaw_Gimbal::TargetAbs(float abs_pitch, float abs_yaw) {
        if (data_.pitch_inverted)
            abs_pitch = -abs_pitch;
        if (data_.upper_yaw_inverted)
            abs_yaw = -abs_yaw;
        float clipped_pitch = clip<float>(abs_pitch, -data_.pitch_max_, data_.pitch_max_);
        pitch_angle_ = wrapping_clip<
            float>(clipped_pitch + data_.pitch_offset_, pitch_lower_limit_, pitch_upper_limit_, 0, 2 * PI);

        // 目标朝向不受 upper_yaw_max_ 限幅：±90° 是上yaw关节相对下yaw的行程，
        // 只要大小yaw协调，云台相对车身/地面可以指向任意角度
        upper_yaw_angle_ = wrapc<float>(abs_yaw + data_.upper_yaw_offset_, 0, 2 * PI);
    }

    void Dual_Yaw_Gimbal::TargetRel(float rel_pitch, float rel_yaw) {
        if (data_.pitch_inverted)
            rel_pitch = -rel_pitch;
        if (data_.upper_yaw_inverted)
            rel_yaw = -rel_yaw;
        pitch_angle_ = wrap<float>(pitch_angle_ + rel_pitch, 0, 2 * PI);
        // 相对量叠加在"目标朝向"上，具体怎么分给两个yaw由 Update* 里的协调逻辑决定
        upper_yaw_angle_ = wrapc<float>(upper_yaw_angle_ + rel_yaw, 0, 2 * PI);
    }

    void Dual_Yaw_Gimbal::TargetReal(float new_pitch, float new_yaw) {
        if (data_.pitch_inverted)
            new_pitch = -new_pitch;
        if (data_.upper_yaw_inverted)
            new_yaw = -new_yaw;
        pitch_angle_ = wrap<float>(pitch_angle_ + new_pitch, 0, 2 * PI);
        upper_yaw_angle_ = wrapc<float>(upper_yaw_angle_ + new_yaw, 0, 2 * PI);
    }

    void Dual_Yaw_Gimbal::UpdateOffset(float pitch_offset, float upper_yaw_offset, float lower_yaw_offset) {
        // 标定零点变化时保持"枪口指向"不变：upper_yaw_angle_ 里含有 upper_yaw_offset_，
        // 所以 offset 增加多少，目标角度也要跟着增加多少（pitch 同理）
        float upper_yaw_relative = upper_yaw_angle_ - data_.upper_yaw_offset_;
        float pitch_relative = pitch_angle_ - data_.pitch_offset_;

        data_.pitch_offset_ = wrap<float>(pitch_offset + data_.pitch_offset_, 0, 2 * PI);
        data_.upper_yaw_offset_ = wrap<float>(upper_yaw_offset + data_.upper_yaw_offset_, 0, 2 * PI);
        data_.lower_yaw_offset_ = wrap<float>(lower_yaw_offset + data_.lower_yaw_offset_, 0, 2 * PI);

        pitch_angle_ = wrapc<float>(pitch_relative + data_.pitch_offset_, 0, 2 * PI);
        upper_yaw_angle_ = wrapc<float>(upper_yaw_relative + data_.upper_yaw_offset_, 0, 2 * PI);

        // pitch 限位是由 offset 算出来的，offset 变了要同步刷新
        pitch_lower_limit_ = wrap<float>(data_.pitch_offset_ - data_.pitch_max_, 0, 2 * PI);
        pitch_upper_limit_ = wrap<float>(data_.pitch_offset_ + data_.pitch_max_, 0, 2 * PI);
    }

    float Dual_Yaw_Gimbal::getPitchTarget() const {
        return pitch_angle_;
    }

    float Dual_Yaw_Gimbal::getUpperYawTarget() const {
        return upper_yaw_angle_;
    }

    float Dual_Yaw_Gimbal::getLowerYawTarget() const {
        return lower_yaw_angle_;
    }

    float Dual_Yaw_Gimbal::getPitchByMotor() const {
        return pitch_motor_->GetOutputShaftCumulatedTheta() - data_.pitch_offset_;
    }

    float Dual_Yaw_Gimbal::getUpperYawByMotor() const {
        // 上yaw编码器测的是它相对下yaw(定子)的角度，减去标定中心就是关节角。
        // 机械行程只有 ±90°，不可能超过一圈，所以 wrap 一次就够；
        // 用多圈累计角是为了不受电机 absolute 模式影响
        float angle = upper_yaw_motor_->GetOutputShaftCumulatedTheta() - data_.upper_yaw_offset_;
        if (data_.upper_yaw_joint_inverted)
            angle = -angle;
        return wrap<float>(angle, -PI, PI);
    }

    float Dual_Yaw_Gimbal::getLowerYawByMotor() const {
        // 下yaw可以连续旋转，同样用多圈累计角，保留圈数
        float angle = lower_yaw_motor_->GetOutputShaftCumulatedTheta() - data_.lower_yaw_offset_;
        if (data_.lower_yaw_joint_inverted)
            angle = -angle;
        return angle;
    }
}  // namespace control
