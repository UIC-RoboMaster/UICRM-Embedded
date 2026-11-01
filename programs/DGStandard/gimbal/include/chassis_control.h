#pragma once

#include <cmath>

#include "remote_mode.h"
#include "yaw_follow_control.h"

namespace dgstandard {

// 底盘指令单位；不是车辆实测的 m/s 或 rad/s。
constexpr float kTranslationAcceleration = 20.0f;
constexpr int kTranslationRockerDeadband = 60;
constexpr float kFollowGain = 8.0f;
constexpr float kFollowMaxCommand = 12.0f;
constexpr float kFollowDeadband = 0.035f;  // 约 2 度以内停止追赶。
constexpr float kFollowAcceleration = 20.0f;
// 小陀螺先固定低速，CH4 不累加速度；目标从零约 1 秒升到 4。
constexpr float kSpinCommand = 4.0f;
constexpr float kSpinAcceleration = 4.0f;
constexpr float kModeChangeDeceleration = 8.0f;

class ChassisTurnControl {
  public:
    void Reset() {
        mode_ = REMOTE_MODE_KILL;
        command_ = 0;
        braking_ = false;
    }

    float Update(RemoteMode mode, float follow_error, float dt) {
        if (!IsActiveRemoteMode(mode)) {
            Reset();
            return 0;
        }
        if (mode != mode_) {
            // 活动模式之间先刹到零，不把 FOLLOW 的高转速/旧方向带进 SPIN。
            braking_ = command_ != 0;
            mode_ = mode;
        }
        if (braking_) {
            command_ = Approach(command_, 0, kModeChangeDeceleration * dt);
            if (command_ == 0)
                braking_ = false;
            return command_;
        }
        if (mode == REMOTE_MODE_SPIN) {
            command_ = Approach(command_, kSpinCommand, kSpinAcceleration * dt);
        } else {
            command_ = FollowTurnCommand(follow_error, command_, dt, kFollowGain,
                                         kFollowMaxCommand, kFollowDeadband, kFollowAcceleration);
        }
        return command_;
    }

    const char* Phase() const {
        if (!IsActiveRemoteMode(mode_))
            return "OFF";
        if (braking_)
            return "BRAKE";
        if (mode_ == REMOTE_MODE_SPIN)
            return command_ == kSpinCommand ? "RUN" : "RAMP";
        return "FOLLOW";
    }

  private:
    static float Approach(float previous, float target, float step) {
        return step <= 0 ? previous : previous + LimitYaw(target - previous, step);
    }

    RemoteMode mode_ = REMOTE_MODE_KILL;
    float command_ = 0;
    bool braking_ = false;
};

struct PlanarVelocity {
    float x;
    float y;
};

// 在枪口坐标中整体限制速度向量的变化，再转换到底盘坐标。
// 从静止加速到固定方向时，横纵分量始终保持比例。
class TranslationRamp {
  public:
    void Reset() { velocity_ = {0, 0}; }

    PlanarVelocity Update(PlanarVelocity target, float dt, float acceleration) {
        if (dt <= 0 || acceleration <= 0)
            return velocity_;
        const float dx = target.x - velocity_.x;
        const float dy = target.y - velocity_.y;
        const float distance_squared = dx * dx + dy * dy;
        const float step = acceleration * dt;
        if (distance_squared <= step * step) {
            velocity_ = target;
        } else {
            const float scale = step / std::sqrt(distance_squared);
            velocity_.x += dx * scale;
            velocity_.y += dy * scale;
        }
        return velocity_;
    }

  private:
    PlanarVelocity velocity_ = {0, 0};
};

// 相对角为 Yaw 编码器减去前向零位；保持已有的平移方向约定。
inline PlanarVelocity RotateVelocity(PlanarVelocity velocity, float sin_angle, float cos_angle) {
    return {cos_angle * velocity.x - sin_angle * velocity.y,
            sin_angle * velocity.x + cos_angle * velocity.y};
}

}  // namespace dgstandard
