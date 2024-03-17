/*###########################################################
 # Copyright (c) 2023-2024. BNU-HKBU UIC RoboMaster         #
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

#include "buzzer_task.h"

#include "bsp_thread.h"
#include "tim.h"

bsp::Thread* buzzer_thread = nullptr;
const osThreadAttr_t buzzer_thread_attr_ = {.name = "BuzzerTask",
                                            .attr_bits = osThreadDetached,
                                            .cb_mem = nullptr,
                                            .cb_size = 0,
                                            .stack_mem = nullptr,
                                            .stack_size = 256 * 4,
                                            .priority = (osPriority_t)osPriorityLow,
                                            .tz_module = 0,
                                            .reserved = 0};

void buzzerTask(void* args);

const bsp::thread_init_t thread_init = {
    .func = buzzerTask,
    .args = nullptr,
    .attr = buzzer_thread_attr_,
};

osThreadId_t buzzerTaskHandle;

driver::Buzzer* buzzer = nullptr;
const driver::BuzzerNoteDelayed* buzzer_song = nullptr;

void Buzzer_Delay(uint32_t delay) {
    osDelay(delay);
}

bool Buzzer_Sing(const driver::BuzzerNoteDelayed* song) {
    if (buzzer_song == nullptr) {
        buzzer_song = song;
        osThreadFlagsSet(buzzerTaskHandle, BUZZER_SIGNAL);
        return true;
    }
    return false;
}
void buzzerTask(void* arg) {
    UNUSED(arg);
    while (1) {
        uint32_t flags = osThreadFlagsWait(BUZZER_SIGNAL, osFlagsWaitAll, osWaitForever);
        if (flags & BUZZER_SIGNAL) {
            if (buzzer_song != nullptr) {
                buzzer->SingSong(buzzer_song, Buzzer_Delay);
                buzzer_song = nullptr;
            }
        }
    }
}

void init_buzzer() {
    buzzer = new driver::Buzzer(&htim4, 3, 1000000);
    buzzer_thread = new bsp::Thread(thread_init);
    buzzer_thread->Start();
}