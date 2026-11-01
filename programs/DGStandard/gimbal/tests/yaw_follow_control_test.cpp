#include "../include/yaw_follow_control.h"
#include "../include/chassis_control.h"

#include <cassert>
#include <cmath>
#include <cstdio>

using dgstandard::WorldYawControl;
using dgstandard::WrapYawError;

constexpr float lead = dgstandard::kYawPi / 12;

static bool near(float a, float b, float tolerance = 0.00002f) {
    return std::fabs(a - b) < tolerance;
}

static float positive(float angle) {
    float wrapped = std::fmod(angle, 2 * dgstandard::kYawPi);
    return wrapped < 0 ? wrapped + 2 * dgstandard::kYawPi : wrapped;
}

static void test_world_reference() {
    WorldYawControl yaw(-1);
    yaw.Reset(0.7f);
    assert(near(yaw.Update(0.7f, 4.0f, 0, 0.001f, 2, lead), 4.0f));
    // 底盘改变了编码器读数，枪口仍指向同一空间方向：不拉回旧编码器目标。
    assert(near(yaw.Update(0.7f, 3.8f, 0, 0.001f, 2, lead), 3.8f));
    assert(near(yaw.Target(), -0.7f));

    // 遥控请求只累加一次；枪转到该方向后，空间误差应归零。
    const float encoder_target = yaw.Update(0.7f, 3.8f, 1, 0.005f, 2, lead);
    assert(near(encoder_target, 3.81f));
    assert(near(yaw.Target(), -0.69f));
    assert(near(yaw.Update(0.69f, 3.81f, 0, 0.001f, 2, lead), 3.81f));
    assert(near(yaw.Error(), 0));

    // 外部扰动和松杆同样受 15 度上限约束。
    const float limited = yaw.Update(1.4f, 3.0f, 0, 0.001f, 2, lead);
    assert(near(WrapYawError(limited - 3.0f), lead));
    assert(near(yaw.Target(), -0.69f));
    // 即使底盘多次把枪口带偏到限幅外，目标方向也必须保留，回到原方向后误差归零。
    for (int i = 0; i < 100; ++i)
        yaw.Update(1.8f, 2.6f, 0, 0.001f, 2, lead);
    assert(near(yaw.Target(), -0.69f));
    yaw.Update(0.69f, 3.81f, 0, 0.001f, 2, lead);
    assert(near(yaw.Error(), 0));
    yaw.Reset(-1.8f);
    assert(near(yaw.Update(-1.8f, 4.9f, 0, 0.001f, 2, lead), 4.9f));
}

static void test_wrap_and_reverse() {
    WorldYawControl yaw(-1);
    yaw.Reset(3.13f);
    yaw.Update(-3.13f, 6.28f, 0, 0.001f, 2, lead);
    assert(near(yaw.Error(), 2 * dgstandard::kYawPi - 6.26f));
    yaw.Reset(0);
    const float wrapped = yaw.Update(0, 6.28f, 1, 0.005f, 2, lead);
    assert(wrapped >= 0 && wrapped < 0.01f);
    for (int i = 0; i < 300; ++i)
        yaw.Update(0, 4.0f, 1, 0.001f, 2, lead);
    assert(near(yaw.Error(), lead));
    yaw.Update(0, 4.0f, 0, 0.001f, 2, lead);
    yaw.Update(0, 4.0f, -1, 0.001f, 2, lead);
    assert(near(yaw.Error(), -0.002f));

    WorldYawControl same_direction(1);
    same_direction.Reset(0.5f);
    same_direction.Update(0.6f, 4.0f, 0, 0.001f, 2, lead);
    assert(near(same_direction.Error(), -0.1f));
}

static float follow(float error, float previous, float dt = 0.008f) {
    return dgstandard::FollowTurnCommand(
        error, previous, dt, dgstandard::kFollowGain, dgstandard::kFollowMaxCommand,
        dgstandard::kFollowDeadband, dgstandard::kFollowAcceleration);
}

static void test_follow_braking() {
    assert(follow(0.5f, 0) > 0);
    assert(follow(-0.5f, 0) < 0);
    assert(near(follow(0.01f, 3), 0));
    // 误差过零后不能继续发送旧方向旋转指令。
    assert(follow(-0.1f, 3) < 0);
    assert(follow(0.1f, -3) > 0);
    assert(follow(0.1f, 3) < 0.6f);
    assert(near(follow(3, 0), 20 * 0.008f));
    assert(near(follow(3, dgstandard::kFollowMaxCommand), dgstandard::kFollowMaxCommand));
    assert(near(follow(3, 0, 0.004f), 20 * 0.004f));
}

// 简化运动模型：用日志估算的底盘速度比例，加入底盘/云台响应滞后。
// 验证参考系与跟随方向，不能代替电机负载、摩擦及真实 PID 的实车验证。
static void test_coupled_follow(float initial_heading, float remote_input, float base_lag) {
    const float dt = 0.001f;
    const float offset = 3.9569f;
    float heading = initial_heading;
    float base = 0;
    float base_rate = 0;
    float encoder_rate = 0;
    float command = 0;
    WorldYawControl yaw(-1);
    yaw.Reset(WrapYawError(-heading));
    for (int i = 0; i < 16000; ++i) {
        const float encoder = positive(heading - base + offset);
        const float input = i < 1000 ? remote_input : 0;
        const float goal = yaw.Update(WrapYawError(-heading), encoder, input, dt, 2, lead);
        assert(std::fabs(WrapYawError(goal - encoder)) <= lead + 0.00001f);
        if (i % 8 == 0)
            command = follow(WrapYawError(encoder - offset), command);
        base_rate += (0.1858f * command - base_rate) * dt / base_lag;
        const float requested_rate = dgstandard::LimitYaw(12 * WrapYawError(goal - encoder), 2);
        encoder_rate += (requested_rate - encoder_rate) * dt / 0.025f;
        base += base_rate * dt;
        heading += (base_rate + encoder_rate) * dt;
    }
    const float final_error = WrapYawError(heading - base);
    assert(std::fabs(final_error) < 0.05f);
    assert(std::fabs(WrapYawError(yaw.Target() - WrapYawError(heading))) < 0.02f);
    assert(std::fabs(command) < 0.05f);
}

int main() {
    test_world_reference();
    test_wrap_and_reverse();
    test_follow_braking();
    test_coupled_follow(1.3f, 0, 0.08f);
    test_coupled_follow(-1.3f, 0, 0.2f);
    test_coupled_follow(3.0f, 0, 0.15f);
    test_coupled_follow(-3.0f, 0, 0.15f);
    test_coupled_follow(0, 1, 0.15f);
    test_coupled_follow(0, -1, 0.15f);
    std::puts("PASS: world heading, sensor signs, angle wrap, rearm, lead limit, reversal, braking and six coupled-follow simulations");
}
