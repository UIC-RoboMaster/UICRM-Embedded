#pragma once

#include "yaw_follow_control.h"

namespace dgstandard {

// 降低角度纠偏增益；保留 2 rad/s 输出上限。补偿速度换算为角度目标偏置，
// 仍经过原有角度环限速及速度环电流限幅，不叠加绕过限幅的输出。
constexpr float kYawAngleGain = 8.0f;
constexpr float kYawMaxRate = 2.0f;
// 共享 PID 的积分按调用次数累计（约 1 kHz），这里不是每秒的积分增益。
constexpr float kYawSpeedIntegralPerTick = 8.0f;
constexpr float kYawMaxOutput = 4000.0f;
constexpr float kYawSpeedFeedbackFilter = 0.25f;
constexpr float kYawCompensationMaxRate = 1.0f;
constexpr float kYawRateWindow = 0.01f;
constexpr float kYawRateFilterTime = 0.05f;

class YawBaseRateEstimator {
  public:
    void Reset(float imu_yaw, float encoder_yaw) {
        previous_imu_ = imu_yaw;
        previous_encoder_ = encoder_yaw;
        elapsed_ = world_delta_ = encoder_delta_ = base_rate_ = 0;
    }

    float Update(float imu_yaw, float encoder_yaw, float dt) {
        if (dt <= 0)
            return base_rate_;
        if (dt > 0.02f) {
            Reset(imu_yaw, encoder_yaw);
            return 0;
        }
        // 实测方向：电机正方向对应 -IMU yaw。
        world_delta_ -= WrapYawError(imu_yaw - previous_imu_);
        encoder_delta_ += WrapYawError(encoder_yaw - previous_encoder_);
        previous_imu_ = imu_yaw;
        previous_encoder_ = encoder_yaw;
        elapsed_ += dt;
        if (elapsed_ >= kYawRateWindow) {
            // 枪口空间转速 = 底盘转速 + 编码器相对转速。
            const float measured_rate = (world_delta_ - encoder_delta_) / elapsed_;
            const float alpha = elapsed_ / (kYawRateFilterTime + elapsed_);
            base_rate_ += alpha * (LimitYaw(measured_rate, 4.0f) - base_rate_);
            elapsed_ = world_delta_ = encoder_delta_ = 0;
        }
        return base_rate_;
    }

    float Compensation() const {
        return -LimitYaw(base_rate_, kYawCompensationMaxRate);
    }

  private:
    float previous_imu_ = 0;
    float previous_encoder_ = 0;
    float elapsed_ = 0;
    float world_delta_ = 0;
    float encoder_delta_ = 0;
    float base_rate_ = 0;
};

inline float CompensatedYawTarget(float encoder_target, float compensation_rate) {
    const float target = encoder_target + LimitYaw(compensation_rate, kYawCompensationMaxRate) / kYawAngleGain;
    return target < 0 ? target + 2 * kYawPi : (target >= 2 * kYawPi ? target - 2 * kYawPi : target);
}

}  // namespace dgstandard
