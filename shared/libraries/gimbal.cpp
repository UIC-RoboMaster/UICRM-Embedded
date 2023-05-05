#include "gimbal.h"

#include "motor.h"
#include "utils.h"

namespace control {

    Gimbal::Gimbal(gimbal_t gimbal)
        : pitch_detector_(BoolEdgeDetector(false)), yaw_detector_(BoolEdgeDetector(false)) {
        // acquired from user
        pitch_motor_ = gimbal.pitch_motor;
        yaw_motor_ = gimbal.yaw_motor;
        data_ = gimbal.data;

        pitch_theta_pid_ = gimbal.pid.pitch_theta_pid;
        pitch_omega_pid_ = gimbal.pid.pitch_omega_pid;
        yaw_theta_pid_ = gimbal.pid.yaw_theta_pid;
        yaw_omega_pid_ = gimbal.pid.yaw_omega_pid;

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

    void Gimbal::SetPID(gimbal_pid_t pid) {
        pitch_theta_pid_ = pid.pitch_theta_pid;
        pitch_theta_pid_->Reset();
        pitch_omega_pid_ = pid.pitch_omega_pid;
        pitch_omega_pid_->Reset();
        yaw_theta_pid_ = pid.yaw_theta_pid;
        yaw_theta_pid_->Reset();
        yaw_omega_pid_ = pid.yaw_omega_pid;
        yaw_omega_pid_->Reset();
    }

    void Gimbal::Update() {
        float pt_diff = pitch_motor_->GetThetaDelta(pitch_angle_);
        pt_diff = wrap<float>(pt_diff, -PI, PI);
        float pt_out = pitch_theta_pid_->ComputeOutput(pt_diff);
        float po_in = pitch_motor_->GetOmegaDelta(pt_out);
        float po_out = pitch_omega_pid_->ComputeConstrainedOutput(po_in);
        float yt_diff = yaw_motor_->GetThetaDelta(yaw_angle_);
        yt_diff = wrap<float>(yt_diff, -PI, PI);
        float yt_out = yaw_theta_pid_->ComputeOutput(yt_diff);
        float yo_in = yaw_motor_->GetOmegaDelta(yt_out);
        float yo_out = yaw_omega_pid_->ComputeConstrainedOutput(yo_in);

        pitch_motor_->SetOutput(po_out);
        yaw_motor_->SetOutput(yo_out);
    }

    void Gimbal::UpdateIMU(float pitch, float yaw) {
        pitch = wrapping_clip<float>(pitch + data_.pitch_offset_, pitch_lower_limit_,
                                     pitch_upper_limit_, 0, 2 * PI);
        yaw = wrapping_clip<float>(yaw + data_.yaw_offset_, yaw_lower_limit_, yaw_upper_limit_, 0,
                                   2 * PI);
        float pt_diff = pitch_angle_ - pitch;
        pt_diff = wrap<float>(pt_diff, -PI, PI);
        float pt_out = pitch_theta_pid_->ComputeOutput(pt_diff);
        float po_in = pitch_motor_->GetOmegaDelta(pt_out);
        float po_out = pitch_omega_pid_->ComputeConstrainedOutput(po_in);

        float yt_diff = yaw_angle_ - yaw;
        yt_diff = wrap<float>(yt_diff, -PI, PI);
        float yt_out = yaw_theta_pid_->ComputeOutput(yt_diff);
        float yo_in = yaw_motor_->GetOmegaDelta(yt_out);
        float yo_out = yaw_omega_pid_->ComputeConstrainedOutput(yo_in);

        pitch_motor_->SetOutput(po_out);
        yaw_motor_->SetOutput(yo_out);
    }

    void Gimbal::TargetAbs(float abs_pitch, float abs_yaw) {
        float clipped_pitch = clip<float>(abs_pitch, -data_.pitch_max_, data_.pitch_max_);
        float clipped_yaw = clip<float>(abs_yaw, -data_.yaw_max_, data_.yaw_max_);
        pitch_angle_ = wrapping_clip<float>(clipped_pitch + data_.pitch_offset_, pitch_lower_limit_,
                                            pitch_upper_limit_, 0, 2 * PI);
        yaw_angle_ = wrapping_clip<float>(clipped_yaw + data_.yaw_offset_, yaw_lower_limit_,
                                          yaw_upper_limit_, 0, 2 * PI);
    }

    void Gimbal::TargetAbsWOffset(float abs_pitch, float abs_yaw) {
        float clipped_pitch = clip<float>(abs_pitch, -data_.pitch_max_, data_.pitch_max_);
        float clipped_yaw = clip<float>(abs_yaw, -data_.yaw_max_, data_.yaw_max_);
        pitch_angle_ =
            wrapping_clip<float>(clipped_pitch, pitch_lower_limit_, pitch_upper_limit_, 0, 2 * PI);
        yaw_angle_ =
            wrapping_clip<float>(clipped_yaw, yaw_lower_limit_, yaw_upper_limit_, 0, 2 * PI);
    }

    void Gimbal::TargetRel(float rel_pitch, float rel_yaw) {
        rel_pitch = clip<float>(rel_pitch, -2 * PI, 2 * PI);
        rel_yaw = clip<float>(rel_yaw, -2 * PI, 2 * PI);
        pitch_angle_ = wrapping_clip<float>(pitch_angle_ + rel_pitch, pitch_lower_limit_,
                                            pitch_upper_limit_, 0, 2 * PI);
        yaw_angle_ = wrapping_clip<float>(yaw_angle_ + rel_yaw, yaw_lower_limit_, yaw_upper_limit_,
                                          0, 2 * PI);
    }

    void Gimbal::UpdateOffset(float pitch_offset, float yaw_offset) {
        data_.pitch_offset_ = wrap<float>(pitch_offset + data_.pitch_offset_, 0, 2 * PI);
        data_.yaw_offset_ = wrap<float>(yaw_offset + data_.yaw_offset_, 0, 2 * PI);
    }

}  // namespace control
