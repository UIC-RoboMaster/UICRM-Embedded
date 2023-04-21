#include "remote_task.h"

remote::DBUS* dbus = nullptr;
RemoteMode remote_mode = REMOTE_MODE_FOLLOW;
RemoteMode last_remote_mode = REMOTE_MODE_FOLLOW;
RemoteMode available_remote_mode[] = {REMOTE_MODE_FOLLOW, REMOTE_MODE_SPIN, REMOTE_MODE_RANDOMSPIN};
const int8_t remote_mode_max = 3;
const int8_t remote_mode_min = 1;
ShootFricMode shoot_fric_mode = SHOOT_FRIC_MODE_STOP;
ShootMode shoot_mode = SHOOT_MODE_STOP;
bool is_killed = false;

void init_dbus() {
    dbus = new remote::DBUS(&huart3);
}
osThreadId_t remoteTaskHandle;
void remoteTask(void* arg) {
    UNUSED(arg);
    osDelay(500);
    unsigned int last_timestamp = dbus->timestamp;
    bool mode_switch = false;
    bool shoot_fric_switch = false;
    bool shoot_switch = false;
    bool shoot_burst_switch = false;
    bool shoot_stop_switch = false;
    uint32_t shoot_burst_timestamp = 0;
    remote::switch_t last_state_r = remote::MID;
    remote::switch_t last_state_l = remote::MID;
    remote::keyboard_t last_keyboard;
    remote::mouse_t last_mouse;
    remote::switch_t state_r = remote::MID;
    remote::switch_t state_l = remote::MID;
    remote::keyboard_t keyboard;
    remote::mouse_t mouse;
    bool is_dbus_offline;
    bool is_robot_dead;
    bool is_shoot_available;
    BoolEdgeDetector* z_edge = new BoolEdgeDetector(false);
    BoolEdgeDetector* ctrl_edge = new BoolEdgeDetector(false);
    BoolEdgeDetector* mouse_left_edge = new BoolEdgeDetector(false);
    BoolEdgeDetector* mouse_right_edge = new BoolEdgeDetector(false);
    while (1) {
        // Update Last State
        last_state_r = state_r;
        last_state_l = state_l;
        last_keyboard = keyboard;
        last_mouse = mouse;
        // Update State
        state_r = dbus->swr;
        state_l = dbus->swl;
        keyboard = dbus->keyboard;
        mouse = dbus->mouse;
        // Update Timestamp
        last_timestamp = dbus->timestamp;
        z_edge->input(dbus->keyboard.bit.Z);
        ctrl_edge->input(dbus->keyboard.bit.CTRL);
        mouse_left_edge->input(mouse.l);
        mouse_right_edge->input(mouse.r);
        // Offline Detection && Security Check
        is_dbus_offline = HAL_GetTick() - last_timestamp > 500 || dbus->swr == remote::DOWN;
        // Kill Detection
        is_robot_dead = referee->game_robot_status.remain_HP == 0;
        is_shoot_available = referee->bullet_remaining.bullet_remaining_num_17mm > 0;
        if (is_dbus_offline || is_robot_dead) {
            if (!is_killed) {
                last_remote_mode = remote_mode;
                remote_mode = REMOTE_MODE_KILL;
                shoot_mode = SHOOT_MODE_DISABLE;
                is_killed = true;
            }
        } else {
            if (is_killed) {
                remote_mode = last_remote_mode;
                shoot_mode = SHOOT_MODE_STOP;
                is_killed = false;
            }
        }
        // when in kill
        if (is_killed) {
            osDelay(REMOTE_OS_DELAY);
            continue;
        }

        if (last_keyboard.bit.G == 1 && keyboard.bit.G == 0) {
            shoot_fric_mode = SHOOT_FRIC_SPEEDUP;
        }
        if (last_keyboard.bit.B == 1 && keyboard.bit.B == 0) {
            shoot_fric_mode = SHOOT_FRIC_SPEEDDOWN;
        }

        // remote mode switch
        switch (state_r) {
            case remote::UP:
                if (last_state_r == remote::MID) {
                    mode_switch = true;
                }
                break;
            case remote::MID:
                if (ctrl_edge->posEdge()) {
                    mode_switch = true;
                }
                break;
            case remote::DOWN:
                break;
        }
        if (mode_switch) {
            mode_switch = false;
            RemoteMode next_mode = (RemoteMode)(remote_mode + 1);
            if ((int8_t)next_mode > (int8_t)remote_mode_max) {
                next_mode = (RemoteMode)remote_mode_min;
            }
            remote_mode = next_mode;
        }
        // shoot mode switch
        if (is_shoot_available == true || SHOOT_REFEREE == 0) {
            switch (state_l) {
                case remote::UP:
                    if (last_state_l == remote::MID) {
                        shoot_fric_switch = true;
                    }
                    break;
                case remote::DOWN:
                    if (last_state_l == remote::MID) {
                        shoot_switch = true;
                        shoot_burst_timestamp = 0;
                    } else if (last_state_l == remote::DOWN) {
                        shoot_burst_timestamp++;
                        if (shoot_burst_timestamp > 500 * REMOTE_OS_DELAY) {
                            shoot_burst_switch = true;
                        }
                    }
                    break;
                case remote::MID:
                    switch (last_state_l) {
                        case remote::DOWN:
                            shoot_stop_switch = true;
                            break;
                        case remote::UP:
                            break;
                        case remote::MID:
                            if (z_edge->posEdge()) {
                                shoot_fric_switch = true;
                            }
                            if (mouse_left_edge->posEdge()) {
                                shoot_switch = true;
                                shoot_burst_timestamp = 0;
                            } else if (mouse_left_edge->get()) {
                                shoot_burst_timestamp++;
                                if (shoot_burst_timestamp > 500 * REMOTE_OS_DELAY) {
                                    shoot_burst_switch = true;
                                }
                            } else if (mouse_left_edge->negEdge()) {
                                shoot_stop_switch = true;
                            } else {
                                shoot_stop_switch = true;
                            }
                            break;
                    }
                    break;
            }
            // 切换摩擦轮模式
            if (shoot_fric_switch) {
                shoot_fric_switch = false;
                if (shoot_fric_mode == SHOOT_FRIC_MODE_STOP) {  // 原来停止则开始转
                    shoot_fric_mode = SHOOT_FRIC_MODE_PREPARING;
                    shoot_mode = SHOOT_MODE_PREPARING;
                } else if (shoot_fric_mode == SHOOT_FRIC_MODE_PREPARED ||
                           shoot_fric_mode == SHOOT_FRIC_MODE_PREPARING) {  // 原来转则停止
                    shoot_fric_mode = SHOOT_FRIC_MODE_STOP;
                    shoot_mode = SHOOT_MODE_STOP;
                }
            }
            // 射出单颗子弹
            if (shoot_switch) {
                shoot_switch = false;
                if (shoot_mode == SHOOT_MODE_PREPARED &&
                    shoot_fric_mode == SHOOT_FRIC_MODE_PREPARED) {
                    // 摩擦轮与拔弹系统准备就绪则发射子弹
                    shoot_mode = SHOOT_MODE_SINGLE;
                }
            }
            // 射出连发子弹
            if (shoot_burst_switch) {
                if (shoot_fric_mode == SHOOT_FRIC_MODE_PREPARED) {
                    // 必须要在准备就绪或者发出单发子弹的情况下才能发射连发子弹
                    if (shoot_mode == SHOOT_MODE_PREPARED || shoot_mode == SHOOT_MODE_SINGLE) {
                        shoot_mode = SHOOT_MODE_BURST;
                        shoot_burst_switch = false;
                    }
                } else {
                    shoot_burst_switch = false;
                }
            }
            // 停止射击
            if (shoot_stop_switch) {
                shoot_stop_switch = false;
                if (shoot_mode == SHOOT_MODE_BURST) {
                    shoot_mode = SHOOT_MODE_PREPARED;
                }
            }
        } else {
            // 没有子弹了，则停止射击
            shoot_mode = SHOOT_MODE_STOP;
            shoot_fric_mode = SHOOT_FRIC_MODE_STOP;
        }
        osDelay(REMOTE_OS_DELAY);
    }
}
void init_remote() {
    init_dbus();
}