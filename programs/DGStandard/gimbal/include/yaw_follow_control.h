#pragma once

#include "remote_control.h"

namespace dgstandard {

constexpr float kYawPi = 3.14159265358979323846f;

// 输入为两次圈内角的差，最多跨一圈。
constexpr float WrapYawError(float value) {
    return value < -kYawPi ? value + 2 * kYawPi :
           (value > kYawPi ? value - 2 * kYawPi : value);
}

constexpr float LimitYaw(float value, float limit) {
    return value > limit ? limit : (value < -limit ? -limit : value);
}

// IMU 随枪口转动：保存空间方向目标，用当前空间误差换算编码器目标。
// imu_to_motor_sign 只描述传感器方向，不改变遥控器方向。
class WorldYawControl {
  public:
    explicit WorldYawControl(float imu_to_motor_sign) : imu_to_motor_sign_(imu_to_motor_sign) {}

    void Reset(float imu_yaw) {
        target_ = imu_to_motor_sign_ * imu_yaw;
        error_ = 0;
        last_direction_ = 0;
    }

    float Update(float imu_yaw, float encoder_yaw, float input, float dt,
                 float remote_speed, float lead_limit) {
        const float measured = imu_to_motor_sign_ * imu_yaw;
        const float error = WrapYawError(target_ - measured);
        const auto step = AdvanceYawTarget(error, LimitYaw(input, 1), dt,
                                           remote_speed, lead_limit, last_direction_);
        last_direction_ = step.direction;
        // 摇杆限制目标领先量；外部扰动只限制本轮纠偏量，不挪走空间目标。
        // 否则底盘把枪口带偏超过 15 度后，松杆仍会不断重写目标并跟着转。
        if (input != 0)
            target_ = WrapYawError(target_ + step.delta);
        error_ = LimitYaw(WrapYawError(target_ - measured), lead_limit);
        const float encoder_target = encoder_yaw + error_;
        return encoder_target < 0 ? encoder_target + 2 * kYawPi :
               (encoder_target >= 2 * kYawPi ? encoder_target - 2 * kYawPi : encoder_target);
    }

    float Target() const { return target_; }
    float Error() const { return error_; }

  private:
    float imu_to_motor_sign_;
    float target_ = 0;
    float error_ = 0;
    int last_direction_ = 0;
};

// 返回最终发送的底盘旋转指令。减速和过零不保留旧方向指令。
inline float FollowTurnCommand(float error, float previous, float dt,
                               float gain, float max_command, float deadband,
                               float acceleration) {
    if (error >= -deadband && error <= deadband)
        return 0;
    const float effective_error = error > 0 ? error - deadband : error + deadband;
    const float target = LimitYaw(gain * effective_error, max_command);
    if (target * previous < 0)
        previous = 0;
    const float target_abs = target < 0 ? -target : target;
    const float previous_abs = previous < 0 ? -previous : previous;
    if (target_abs <= previous_abs)
        return target;
    return previous + LimitYaw(target - previous, acceleration * dt);
}

}  // namespace dgstandard
