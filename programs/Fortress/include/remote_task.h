#pragma once
#include "cmsis_os2.h"
#include "dbus.h"
#include "main.h"
#include "user_define.h"
extern remote::DBUS* dbus;
void init_dbus();
enum RemoteMode {
    REMOTE_MODE_PREPARE = -2,
    REMOTE_MODE_KILL = -1,
    REMOTE_MODE_STOP = 0,
    REMOTE_MODE_MANUAL = 1,
    REMOTE_MODE_SPIN = 2,

};
extern RemoteMode remote_mode;
extern osThreadId_t remoteTaskHandle;
const osThreadAttr_t remoteTaskAttribute = {.name = "remoteTask",
                                            .attr_bits = osThreadDetached,
                                            .cb_mem = nullptr,
                                            .cb_size = 0,
                                            .stack_mem = nullptr,
                                            .stack_size = 512 * 4,
                                            .priority = (osPriority_t)osPriorityHigh,
                                            .tz_module = 0,
                                            .reserved = 0};
void remoteTask(void* arg);
void init_remote();