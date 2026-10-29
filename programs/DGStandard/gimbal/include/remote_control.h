#pragma once

namespace dgstandard {

// 中心死区归零，死区外从 0 平滑映射到满杆 1，避免跨过死区瞬间的阶跃。
constexpr float NormalizeRocker(int raw, int maximum, int deadband) {
    if (maximum <= 0 || (raw >= -deadband && raw <= deadband))
        return 0;
    if (raw > deadband)
        return float(raw - deadband) / float(maximum - deadband);
    return float(raw + deadband) / float(maximum - deadband);
}

struct YawTargetStep {
    float delta;
    int direction;
    bool reset_integral;
};

// error 必须是目标减实际位置的最短角度差。返回值用于 Gimbal::TargetRel。
constexpr YawTargetStep AdvanceYawTarget(float error, float input, float dt,
                                       float speed, float lead_limit, int last_direction) {
    if (input == 0)
        return {0, last_direction, false};
    const int direction = input > 0 ? 1 : -1;
    const bool reversed = last_direction != 0 && direction != last_direction;
    // 反向时从实际位置重新起算，避免先花几秒退回积压的旧目标。
    const float requested_error = (reversed ? 0 : error) + input * speed * dt;
    const float bounded_error = requested_error > lead_limit ? lead_limit :
                                (requested_error < -lead_limit ? -lead_limit : requested_error);
    return {bounded_error - error, direction, reversed};
}

}  // namespace dgstandard
