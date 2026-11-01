#include "../include/chassis_control.h"
#include "../include/yaw_stabilization.h"

#include <cassert>
#include <cmath>
#include <cstdio>
#include <cstring>

using dgstandard::ChassisTurnControl;
using dgstandard::SelectRemoteMode;
using dgstandard::WorldYawControl;
using dgstandard::WrapYawError;

static bool near(float a, float b, float tolerance = 0.00002f) {
    return std::fabs(a - b) < tolerance;
}

static float positive(float value) {
    const float angle = std::fmod(value, 2 * dgstandard::kYawPi);
    return angle < 0 ? angle + 2 * dgstandard::kYawPi : angle;
}

static void test_remote_modes() {
    assert(SelectRemoteMode(true, true, false) == REMOTE_MODE_SPIN);
    assert(SelectRemoteMode(true, false, true) == REMOTE_MODE_FOLLOW);
    assert(SelectRemoteMode(true, false, false) == REMOTE_MODE_KILL);
    assert(SelectRemoteMode(true, true, true) == REMOTE_MODE_KILL);
    for (int up = 0; up < 2; ++up)
        for (int mid = 0; mid < 2; ++mid)
            assert(SelectRemoteMode(false, up, mid) == REMOTE_MODE_KILL);
    assert(dgstandard::IsActiveRemoteMode(REMOTE_MODE_FOLLOW));
    assert(dgstandard::IsActiveRemoteMode(REMOTE_MODE_SPIN));
    assert(!dgstandard::IsActiveRemoteMode(REMOTE_MODE_KILL));
    assert(!dgstandard::IsActiveRemoteMode(REMOTE_MODE_STOP));
    assert(!dgstandard::IsActiveRemoteMode(REMOTE_MODE_ADVANCED));
}

static void test_spin_ramp_and_reset() {
    ChassisTurnControl control;
    assert(std::strcmp(control.Phase(), "OFF") == 0);
    float command = control.Update(REMOTE_MODE_SPIN, 2, 0.006f);
    assert(near(command, 0.024f));
    assert(std::strcmp(control.Phase(), "RAMP") == 0);
    assert(near(control.Update(REMOTE_MODE_SPIN, -2, 0), command));
    for (int i = 0; i < 200; ++i) {
        const float previous = command;
        command = control.Update(REMOTE_MODE_SPIN, (i % 2 == 0 ? -3.1f : 3.1f), 0.006f);
        assert(command >= previous);
        assert(command - previous <= dgstandard::kSpinAcceleration * 0.006f + 0.00001f);
        assert(command <= dgstandard::kSpinCommand);
    }
    assert(near(command, dgstandard::kSpinCommand));
    assert(std::strcmp(control.Phase(), "RUN") == 0);
    // 下档或掉线直接停机，重新进入不能保留旧转速。
    assert(near(control.Update(SelectRemoteMode(false, true, false), 2, 0.006f), 0));
    assert(std::strcmp(control.Phase(), "OFF") == 0);
    assert(near(control.Update(REMOTE_MODE_SPIN, 0, 0.006f), 0.024f));
    control.Reset();
    assert(near(control.Update(REMOTE_MODE_FOLLOW, 0, 0.006f), 0));

    ChassisTurnControl a;
    ChassisTurnControl b;
    float first = 0;
    float second = 0;
    for (int i = 0; i < 100; ++i)
        first = a.Update(REMOTE_MODE_SPIN, 0, 0.006f);
    for (int i = 0; i < 200; ++i)
        second = b.Update(REMOTE_MODE_SPIN, 0, 0.003f);
    assert(near(first, second));
    assert(near(first, 2.4f));
}

static void test_brake_between_modes(float follow_error) {
    ChassisTurnControl control;
    float command = 0;
    for (int i = 0; i < 200; ++i)
        command = control.Update(REMOTE_MODE_FOLLOW, follow_error, 0.006f);
    assert(std::fabs(command) > dgstandard::kSpinCommand);
    bool reached_zero = false;
    for (int i = 0; i < 1000; ++i) {
        const float previous = command;
        command = control.Update(REMOTE_MODE_SPIN, -follow_error, 0.006f);
        if (!reached_zero) {
            assert(std::fabs(command) <= std::fabs(previous));
            assert(std::fabs(command - previous) <= dgstandard::kModeChangeDeceleration * 0.006f + 0.00001f);
            reached_zero = command == 0;
        } else {
            assert(command >= 0 && command <= dgstandard::kSpinCommand);
        }
    }
    assert(reached_zero && near(command, dgstandard::kSpinCommand));

    reached_zero = false;
    for (int i = 0; i < 1000; ++i) {
        const float previous = command;
        command = control.Update(REMOTE_MODE_FOLLOW, follow_error, 0.006f);
        if (!reached_zero) {
            assert(command >= 0 && command <= previous);
            reached_zero = command == 0;
        }
    }
    assert(reached_zero);
    assert(command * follow_error > 0);
    assert(std::fabs(command) <= dgstandard::kFollowMaxCommand);
}

// 简化底盘/云台响应模型：检查连续跨编码器零位及切换时空间目标不重置。
// 电机扭矩、真实 PID 和车辆负载仍需实车验证。
static void test_heading_during_spin_and_follow(float initial_heading, float base_lag, float remote_input) {
    const float dt = 0.001f;
    const float lead = dgstandard::kYawPi / 12;
    const float offset = 3.9569f;
    float heading = initial_heading;
    float base = 0;
    float base_rate = 0;
    float encoder_rate = 0;
    float command = 0;
    float peak_relative_rotation = 0;
    WorldYawControl yaw(-1);
    dgstandard::YawBaseRateEstimator base_estimator;
    ChassisTurnControl turn;
    yaw.Reset(WrapYawError(-heading));
    base_estimator.Reset(WrapYawError(-heading), positive(heading + offset));
    for (int i = 0; i < 32000; ++i) {
        const bool spinning = i >= 3000 && i < 23000;
        const RemoteMode mode = spinning ? REMOTE_MODE_SPIN : REMOTE_MODE_FOLLOW;
        const float encoder = positive(heading - base + offset);
        const float input = i >= 8000 && i < 9000 ? remote_input : 0;
        const float target_before = yaw.Target();
        const float heading_goal = yaw.Update(WrapYawError(-heading), encoder, input, dt, 2, lead);
        assert(std::fabs(WrapYawError(heading_goal - encoder)) <= lead + 0.00001f);
        base_estimator.Update(WrapYawError(-heading), encoder, dt);
        const float goal = dgstandard::CompensatedYawTarget(heading_goal, base_estimator.Compensation());
        if (i == 3000 || i == 23000)
            assert(near(WrapYawError(yaw.Target() - target_before), 0));
        if (i % 6 == 0)
            command = turn.Update(mode, WrapYawError(encoder - offset), 0.006f);
        base_rate += (0.1858f * command - base_rate) * dt / base_lag;
        const float requested_rate = dgstandard::LimitYaw(
            dgstandard::kYawAngleGain * WrapYawError(goal - encoder), dgstandard::kYawMaxRate);
        encoder_rate += (requested_rate - encoder_rate) * dt / 0.025f;
        base += base_rate * dt;
        heading += (base_rate + encoder_rate) * dt;
        if (spinning && std::fabs(base - heading) > peak_relative_rotation)
            peak_relative_rotation = std::fabs(base - heading);
        if (i > 11000 && i < 23000)
            assert(std::fabs(WrapYawError(heading - yaw.Target())) < 0.01f);
    }
    assert(peak_relative_rotation > 4 * dgstandard::kYawPi);
    // base 累计超过两圈；先映射回圈内，不能把多圈差直接传给 WrapYawError。
    assert(std::fabs(WrapYawError(positive(heading - base))) < 0.05f);
    assert(std::fabs(WrapYawError(heading - yaw.Target())) < 0.02f);
    assert(std::fabs(command) < 0.05f);
    // 返回 FOLLOW 的瞬态可能触及 15 度领先限幅；检查空间目标保持在请求附近。
    assert(near(WrapYawError(yaw.Target() - WrapYawError(initial_heading + 2 * remote_input)), 0, 0.03f));
}

int main() {
    test_remote_modes();
    test_spin_ramp_and_reset();
    test_brake_between_modes(2);
    test_brake_between_modes(-2);
    test_heading_during_spin_and_follow(0, 0.08f, 0);
    test_heading_during_spin_and_follow(3.1f, 0.2f, 0.25f);
    test_heading_during_spin_and_follow(-3.1f, 0.15f, -0.25f);
    std::puts("PASS: switch mapping, spin speed/ramp, mode braking, kill/rearm, world heading across multiple turns and return to FOLLOW");
}
