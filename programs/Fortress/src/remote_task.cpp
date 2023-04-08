#include "remote_task.h"

remote::DBUS* dbus = nullptr;
RemoteMode remote_mode = REMOTE_MODE_MANUAL;
RemoteMode last_remote_mode = REMOTE_MODE_MANUAL;
RemoteMode available_remote_mode[] = {REMOTE_MODE_MANUAL, REMOTE_MODE_SPIN, REMOTE_MODE_ADVANCED};
const int8_t remote_mode_max = 3;
const int8_t remote_mode_min = 1;
bool is_killed = false;
void init_dbus() {
    dbus = new remote::DBUS(&huart3);
}
osThreadId_t remoteTaskHandle;
void remoteTask(void* arg) {
    UNUSED(arg);
    osDelay(500);
    unsigned int last_timestamp = dbus->timestamp;
    remote::switch_t last_state = dbus->swr;
    bool mode_switch = false;
    while (1) {
        last_timestamp = dbus->timestamp;
        // Offline Detection
        bool is_dbus_offline = HAL_GetTick() - last_timestamp > 500 || dbus->swr == remote::DOWN;
        // Kill Detection
        bool is_robot_dead = referee->game_robot_status.remain_HP == 0;
        if (is_dbus_offline || is_robot_dead) {
            if (!is_killed) {
                last_remote_mode = remote_mode;
                remote_mode = REMOTE_MODE_KILL;
                is_killed = true;
            }
        } else {
            if (is_killed) {
                remote_mode = last_remote_mode;
                is_killed = false;
            }
        }
        // TODO refree death detection
        // when in kill
        if (is_killed) {
            osDelay(REMOTE_OS_DELAY);
            continue;
        }
        // remote mode switch
        if (dbus->swr == remote::UP) {
            if (last_state == remote::MID) {
                mode_switch = true;
            }
            last_state = remote::UP;
        } else if (dbus->swr == remote::MID) {
            if (last_state == remote::UP) {}
            last_state = remote::MID;
        }
        if (mode_switch) {
            mode_switch = false;
            RemoteMode next_mode = (RemoteMode)(remote_mode + 1);
            if ((int8_t)next_mode > (int8_t)remote_mode_max) {
                next_mode = (RemoteMode)remote_mode_min;
            }
            remote_mode = next_mode;
        }
        osDelay(REMOTE_OS_DELAY);
    }
}
void init_remote() {
    init_dbus();
}