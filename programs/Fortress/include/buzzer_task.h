/*###########################################################
 # Copyright (c) 2023. BNU-HKBU UIC RoboMaster              #
 #                                                          #
 # This program is free software: you can redistribute it   #
 # and/or modify it under the terms of the GNU General      #
 # Public License as published by the Free Software         #
 # Foundation, either version 3 of the License, or (at      #
 # your option) any later version.                          #
 #                                                          #
 # This program is distributed in the hope that it will be  #
 # useful, but WITHOUT ANY WARRANTY; without even           #
 # the implied warranty of MERCHANTABILITY or FITNESS       #
 # FOR A PARTICULAR PURPOSE.  See the GNU General           #
 # Public License for more details.                         #
 #                                                          #
 # You should have received a copy of the GNU General       #
 # Public License along with this program.  If not, see     #
 # <https://www.gnu.org/licenses/>.                         #
 ###########################################################*/

#pragma once
#include "buzzer.h"
#include "cmsis_os2.h"
#include "main.h"

#define BUZZER_SIGNAL (1 << 0)

extern driver::Buzzer* buzzer;

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
bool Buzzer_Sing(const driver::BuzzerNoteDelayed* song);
void buzzerTask(void* arg);
void init_buzzer();
