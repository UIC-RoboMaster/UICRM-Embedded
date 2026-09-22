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

        // 朝向死区：误差很小时不再修正，避免在目标附近抖动
        if (abs(yaw_diff) < data_.upper_yaw_deadband) {
            yaw_diff = 0;
        }

        float current_upper_angle = getUpperYawByMotor();
        float current_lower_angle = getLowerYawByMotor();
        float upper_encoder = upper_yaw_motor_->GetOutputShaftCumulatedTheta();
        float lower_encoder = lower_yaw_motor_->GetOutputShaftCumulatedTheta();

        // θ* = 目标地面朝向，θ = 当前地面朝向
        // yaw_diff = θ* − θ
        // 小yaw电机角度 u，大yaw电机角度 l

        // 大yaw：从第一周期起就朝"最终由它承担的朝向"走，即让下yaw轴角度 l 趋近目标角度 θ。
        // 因为 θ = u + l，所以 θ* - l = u_now + yaw_diff。

        // 用 lower_yaw_recenter_max_step 限速，决定交接（回中）的快慢；
        // 取 0 表示下yaw不主动接手（只补上yaw顶限位的差额）。
        float lower_step = 0.0f;
        if (data_.lower_yaw_recenter_max_step > 0.0f) {
            lower_step = clip<float>(wrapc<float>(current_upper_angle + yaw_diff, -PI, PI),
                                     -data_.lower_yaw_recenter_max_step,
                                     data_.lower_yaw_recenter_max_step);
        }

        // 小yaw：补上大yaw还没到位的部分，使大小yaw转动之和 = 本周期移动 yaw_diff，
        float upper_diff = yaw_diff - lower_step;
        float upper_target = current_upper_angle + upper_diff;
        // 如果
        if (!data_.upper_yaw_circle_) {
            upper_target = clip<float>(upper_target, -data_.upper_yaw_max_, data_.upper_yaw_max_);
        }
        // 更新 upper_diff
        upper_diff = upper_target - current_upper_angle;
        upper_yaw_motor_->SetTarget(upper_encoder + sign_upper * upper_diff);

        // 小yaw顶到限位时，剩下的差额由大yaw补
        lower_step = yaw_diff - upper_diff;

        // 下yaw自身限位（能连续旋转时不需要）
        if (!data_.lower_yaw_circle_) {
            float lower_cmd = clip<float>(current_lower_angle + lower_step, -data_.lower_yaw_max_, data_.lower_yaw_max_);
            lower_step = lower_cmd - current_lower_angle;
        }

        // 下yaw死区。注意它同时作用于回中量：回中步长要明显大于该死区，否则回中会被吃掉
        if (abs(lower_step) < data_.lower_yaw_deadband) {
            lower_step = 0;
        }

        lower_yaw_angle_ = lower_encoder + sign_lower * lower_step;
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
