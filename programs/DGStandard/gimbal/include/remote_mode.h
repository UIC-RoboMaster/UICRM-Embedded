#pragma once

// 保留现有枚举值，供遥控任务、各执行任务和串口显示共同使用。
enum RemoteMode {
    REMOTE_MODE_PREPARE = -2,
    REMOTE_MODE_KILL = -1,
    REMOTE_MODE_STOP = 0,
    REMOTE_MODE_FOLLOW = 1,
    REMOTE_MODE_SPIN = 2,
    REMOTE_MODE_ADVANCED = 3,
    REMOTE_MODE_AUTOAIM = 4,
};

inline const char* remote_mode_str(RemoteMode mode) {
    switch (mode) {
        case REMOTE_MODE_PREPARE: return "PREPARE";
        case REMOTE_MODE_KILL: return "KILL";
        case REMOTE_MODE_STOP: return "STOP";
        case REMOTE_MODE_FOLLOW: return "FOLLOW";
        case REMOTE_MODE_SPIN: return "SPIN";
        case REMOTE_MODE_ADVANCED: return "ADVANCED";
        case REMOTE_MODE_AUTOAIM: return "AUTOAIM";
        default: return "UNKNOWN";
    }
}

namespace dgstandard {

constexpr bool IsActiveRemoteMode(RemoteMode mode) {
    return mode == REMOTE_MODE_FOLLOW || mode == REMOTE_MODE_SPIN;
}

// 右拨杆：上档 SPIN、中档 FOLLOW；下档、无效档位、掉线均 KILL。
constexpr RemoteMode SelectRemoteMode(bool online, bool right_up, bool right_mid) {
    return !online || right_up == right_mid ? REMOTE_MODE_KILL :
           (right_up ? REMOTE_MODE_SPIN : REMOTE_MODE_FOLLOW);
}

}  // namespace dgstandard
