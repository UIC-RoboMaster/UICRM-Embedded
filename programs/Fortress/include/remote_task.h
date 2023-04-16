#pragma once
#include "cmsis_os2.h"
#include "dbus.h"
#include "main.h"
#include "referee_task.h"
#include "user_define.h"
extern remote::DBUS* dbus;
/**
 * @brief Dbus Init
 */
void init_dbus();

enum RemoteMode {
    REMOTE_MODE_PREPARE = -2,
    REMOTE_MODE_KILL = -1,
    REMOTE_MODE_STOP = 0,
    REMOTE_MODE_FOLLOW = 1,
    REMOTE_MODE_SPIN = 2,
    REMOTE_MODE_ADVANCED = 3
};
extern RemoteMode remote_mode;
enum ShootFricMode {
    SHOOT_FRIC_MODE_DISABLE = -1,
    SHOOT_FRIC_MODE_STOP = 0,
    SHOOT_FRIC_MODE_PREPARING = 1,
    SHOOT_FRIC_MODE_PREPARED = 2,
    SHOOT_FRIC_SPEEDUP = 3,
    SHOOT_FRIC_SPEEDDOWN = 4,
};
extern ShootFricMode shoot_fric_mode;
enum ShootMode {
    SHOOT_MODE_DISABLE = -1,
    SHOOT_MODE_STOP = 0,
    SHOOT_MODE_PREPARING = 1,
    SHOOT_MODE_PREPARED = 2,
    SHOOT_MODE_SINGLE = 3,
    SHOOT_MODE_BURST = 4,
};
extern ShootMode shoot_mode;
extern osThreadId_t remoteTaskHandle;
const osThreadAttr_t remoteTaskAttribute = {.name = "remoteTask",
                                            .attr_bits = osThreadDetached,
                                            .cb_mem = nullptr,
                                            .cb_size = 0,
                                            .stack_mem = nullptr,
                                            .stack_size = 512 * 4,
                                            .priority = (osPriority_t)osPriorityHigh7,
                                            .tz_module = 0,
                                            .reserved = 0};
void remoteTask(void* arg);
void init_remote();