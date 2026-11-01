#include "../include/chassis_control.h"
#include "../include/remote_control.h"

#include <cassert>
#include <cmath>
#include <cstdio>

using dgstandard::PlanarVelocity;
using dgstandard::TranslationRamp;

static bool near(float a, float b, float tolerance = 0.00002f) {
    return std::fabs(a - b) < tolerance;
}

static float length(PlanarVelocity value) {
    return std::sqrt(value.x * value.x + value.y * value.y);
}

static PlanarVelocity rotate(PlanarVelocity velocity, float angle) {
    return dgstandard::RotateVelocity(velocity, std::sin(angle), std::cos(angle));
}

static void test_straight_acceleration_and_stop() {
    TranslationRamp ramp;
    const PlanarVelocity target = {5, 8.660254f};
    PlanarVelocity velocity = {0, 0};
    for (int i = 0; i < 200; ++i) {
        velocity = ramp.Update(target, 0.006f, dgstandard::kTranslationAcceleration);
        // 启动到稳态均沿目标方向，而不是先沿 45 度方向走。
        assert(std::fabs(velocity.x * target.y - velocity.y * target.x) < 0.0002f);
        assert(length(velocity) <= length(target) + 0.0001f);
    }
    assert(near(velocity.x, target.x));
    assert(near(velocity.y, target.y));
    for (int i = 0; i < 200; ++i) {
        velocity = ramp.Update({0, 0}, 0.006f, dgstandard::kTranslationAcceleration);
        assert(std::fabs(velocity.x * target.y - velocity.y * target.x) < 0.0002f);
    }
    assert(near(length(velocity), 0));
}

static void test_fixed_gun_rotating_chassis() {
    TranslationRamp ramp;
    const float gun_heading = 0.4f;
    const PlanarVelocity target = {0, 10};
    const auto expected_world = rotate(target, gun_heading);
    for (int i = 0; i < 600; ++i) {
        const auto gun = ramp.Update(target, 0.006f, dgstandard::kTranslationAcceleration);
        const float relative_yaw = 3.3f - 0.011f * i;
        // 枪口保持空间方向，底盘持续转动，包含跨 +/-PI 的情况。
        const float base_heading = gun_heading - relative_yaw;
        const auto body = rotate(gun, relative_yaw);
        const auto world = rotate(body, base_heading);
        assert(std::fabs(world.x * expected_world.y - world.y * expected_world.x) < 0.0002f);
        // 回到枪口坐标必须仍然只有前进分量。
        const auto restored = rotate(body, -relative_yaw);
        assert(near(restored.x, 0));
        assert(near(restored.y, gun.y));
    }
}

static void test_time_step_reversal_and_reset() {
    TranslationRamp slow;
    TranslationRamp fast;
    PlanarVelocity a = {0, 0};
    PlanarVelocity b = {0, 0};
    for (int i = 0; i < 100; ++i)
        a = slow.Update({30, 0}, 0.006f, dgstandard::kTranslationAcceleration);
    for (int i = 0; i < 200; ++i)
        b = fast.Update({30, 0}, 0.003f, dgstandard::kTranslationAcceleration);
    assert(near(a.x, b.x, 0.0001f));
    assert(near(a.x, 12, 0.0001f));
    for (int i = 0; i < 500; ++i) {
        const PlanarVelocity target = i < 250 ? PlanarVelocity{-30, 0} : PlanarVelocity{0, 30};
        b = slow.Update(target, 0.006f, dgstandard::kTranslationAcceleration);
        assert(length({b.x - a.x, b.y - a.y}) <= 0.12001f);
        if (i < 250)
            assert(near(b.y, 0));
        a = b;
    }
    slow.Reset();
    a = slow.Update({0, 0}, 0.006f, dgstandard::kTranslationAcceleration);
    assert(near(length(a), 0));
    a = slow.Update({0, 30}, 0, dgstandard::kTranslationAcceleration);
    assert(near(length(a), 0));
    a = slow.Update({0, 30}, 0.006f, dgstandard::kTranslationAcceleration);
    assert(near(length(a), 0.12f));
}

static void test_rocker_deadband() {
    using dgstandard::NormalizeRocker;
    const int deadband = dgstandard::kTranslationRockerDeadband;
    assert(near(NormalizeRocker(-50, 660, deadband), 0));
    assert(near(NormalizeRocker(50, 660, deadband), 0));
    assert(near(NormalizeRocker(deadband, 660, deadband), 0));
    assert(near(NormalizeRocker(660, 660, deadband), 1));
    assert(near(NormalizeRocker(-660, 660, deadband), -1));
    assert(NormalizeRocker(deadband + 1, 660, deadband) > 0);
}

int main() {
    test_straight_acceleration_and_stop();
    test_fixed_gun_rotating_chassis();
    test_time_step_reversal_and_reset();
    test_rocker_deadband();
    std::puts("PASS: straight acceleration/braking, fixed-gun translation during chassis rotation, time steps, reversal, rearm and rocker deadband");
}
