#pragma once
#include "adernal_supercap.h"
#include "cmsis_os2.h"

extern osThreadId_t capacityTaskHandle;
const osThreadAttr_t capacityTaskAttribute = {.name = "capacityTask",
                                           .attr_bits = osThreadDetached,
                                           .cb_mem = nullptr,
                                           .cb_size = 0,
                                           .stack_mem = nullptr,
                                           .stack_size = 1024 * 4,
                                           .priority = (osPriority_t)osPriorityNormal,
                                           .tz_module = 0,
                                           .reserved = 0};

extern driver::Adernal_SuperCap* adernal_supercap;
extern driver::Adernal_StdbyState_Typedef supercap_standby;
extern driver::Adernal_Fb_Typedef supercap_feedback;
extern const driver::Adernal_SafetyLevel_Typedef* supercap_safety_levels;
extern float supercap_remaining;
void init_capacity();
void capacityTask(void* arg);
void silentMode();
void chargeMode();
void workMode();