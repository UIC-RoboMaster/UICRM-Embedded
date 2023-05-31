#pragma once
#include "buzzer_notes.h"
#include "buzzer_task.h"
#include "chassis_task.h"
#include "cmsis_os2.h"
#include "gimbal_task.h"
#include "imu_task.h"
#include "main.h"
#include "motor.h"
#include "referee_task.h"
#include "remote_task.h"
#include "shoot_task.h"
#include "user_define.h"

extern osThreadId_t selftestTaskHandle;
const osThreadAttr_t selftestTaskAttribute = {.name = "selftestTask",
                                              .attr_bits = osThreadDetached,
                                              .cb_mem = nullptr,
                                              .cb_size = 0,
                                              .stack_mem = nullptr,
                                              .stack_size = 256 * 4,
                                              .priority = (osPriority_t)osPriorityBelowNormal,
                                              .tz_module = 0,
                                              .reserved = 0};

typedef struct {
    bool fl_motor;
    bool fr_motor;
    bool bl_motor;
    bool br_motor;
    bool yaw_motor;
    bool pitch_motor;
    bool steering_motor;
    bool dbus;
    bool referee;
    bool refereerc;
    bool imu_cali;
    bool imu_temp;
    bool gimbal;
    bool chassis;
    bool shoot;
    bool supercap;
    bool can_bus;
} selftest_t;

extern selftest_t selftest;

void init_selftest();
void selftestTask(void* arg);
