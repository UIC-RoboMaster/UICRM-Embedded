#include "../include/yaw_stabilization.h"

#include <cassert>
#include <cmath>
#include <cstdio>

static bool near(float a, float b, float tolerance = 0.0001f) {
    return std::fabs(a - b) < tolerance;
}

static float positive(float value) {
    const float angle = std::fmod(value, 2 * dgstandard::kYawPi);
    return angle < 0 ? angle + 2 * dgstandard::kYawPi : angle;
}

static void test_sensor_rates(float base_rate, float world_rate, float dt) {
    dgstandard::YawBaseRateEstimator estimator;
    float heading = 3.13f;
    float encoder = 0.02f;
    estimator.Reset(-heading, encoder);
    float estimated = 0;
    for (int i = 0; i < static_cast<int>(3 / dt); ++i) {
        heading += world_rate * dt;
        encoder += (world_rate - base_rate) * dt;
        estimated = estimator.Update(dgstandard::WrapYawError(-heading), positive(encoder), dt);
    }
    assert(near(estimated, base_rate, 0.002f));
    assert(near(estimator.Compensation(), -dgstandard::LimitYaw(base_rate, 1), 0.002f));
    assert(near(estimator.Update(dgstandard::WrapYawError(-heading), positive(encoder), 0), estimated));
    // 掉线重启或过长采样间隔清掉旧转速，不能带着旧补偿解锁。
    estimator.Reset(0, 1);
    assert(near(estimator.Compensation(), 0));
    assert(near(estimator.Update(1, 3, 0.1f), 0));
}

static void test_encoder_bias() {
    const float bias = dgstandard::kYawCompensationMaxRate / dgstandard::kYawAngleGain;
    assert(near(dgstandard::CompensatedYawTarget(4, 0), 4));
    assert(near(dgstandard::CompensatedYawTarget(4, 100), 4 + bias));
    assert(near(dgstandard::CompensatedYawTarget(4, -100), 4 - bias));
    assert(dgstandard::CompensatedYawTarget(0.01f, -1) > 6);
    assert(near(dgstandard::CompensatedYawTarget(6.28f, 1), positive(6.28f + bias)));
}

static float simulated_error(bool compensate) {
    dgstandard::WorldYawControl world(-1);
    dgstandard::YawBaseRateEstimator estimator;
    float heading = 0;
    float base = 0;
    float relative_rate = 0;
    const float dt = 0.001f;
    const float offset = 3.9830f;
    const float lead = dgstandard::kYawPi / 12;
    world.Reset(0);
    estimator.Reset(0, offset);
    float error_sum = 0;
    for (int i = 0; i < 12000; ++i) {
        const float encoder = positive(heading - base + offset);
        const float heading_goal = world.Update(-heading, encoder, 0, dt, 2, lead);
        estimator.Update(-heading, encoder, dt);
        const float goal = dgstandard::CompensatedYawTarget(
            heading_goal, compensate ? estimator.Compensation() : 0);
        const float requested = dgstandard::LimitYaw(
            dgstandard::kYawAngleGain * dgstandard::WrapYawError(goal - encoder), 2);
        relative_rate += (requested - relative_rate) * dt / 0.025f;
        // 稳态底盘转动，模拟速度低于现有云台 2 rad/s 补偿能力。
        base += 0.74f * dt;
        heading += (0.74f + relative_rate) * dt;
        if (i > 6000)
            error_sum += std::fabs(heading);
    }
    return error_sum / 5999;
}

int main() {
    test_sensor_rates(0, 0.8f, 0.001f);  // 仅枪口转动，不能误判成底盘转动。
    test_sensor_rates(0.74f, 0, 0.001f);
    test_sensor_rates(-0.74f, 0, 0.002f);
    test_sensor_rates(0.74f, 0.3f, 0.001f);
    test_sensor_rates(2.0f, 0, 0.001f);  // 大扰动时补偿上限仍为 1 rad/s。
    test_encoder_bias();
    const float before = simulated_error(false);
    const float after = simulated_error(true);
    assert(after < before / 10);
    std::printf("PASS: sensor sign/wrap, rate estimate, rearm, compensation bounds; simple model heading error %.4f -> %.4f rad\n", before, after);
}
