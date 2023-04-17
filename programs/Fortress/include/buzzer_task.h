#pragma once
#include "bsp_buzzer.h"
#include "cmsis_os2.h"
#include "main.h"

#define BUZZER_SIGNAL (1 << 0)

extern bsp::Buzzer* buzzer;

extern osThreadId_t buzzerTaskHandle;
const osThreadAttr_t buzzerTaskAttribute = {.name = "buzzerTask",
                                            .attr_bits = osThreadDetached,
                                            .cb_mem = nullptr,
                                            .cb_size = 0,
                                            .stack_mem = nullptr,
                                            .stack_size = 128 * 4,
                                            .priority = (osPriority_t)osPriorityBelowNormal,
                                            .tz_module = 0,
                                            .reserved = 0};
bool Buzzer_Sing(const bsp::BuzzerNoteDelayed* song);
void buzzerTask(void* arg);
void init_buzzer();
