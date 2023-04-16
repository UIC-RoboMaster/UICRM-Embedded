#include "buzzer_task.h"

osThreadId_t buzzerTaskHandle;

bsp::Buzzer* buzzer = nullptr;
const bsp::BuzzerNoteDelayed* buzzer_song = nullptr;

void Buzzer_Delay(uint32_t delay) {
    osDelay(delay);
}


bool Buzzer_Sing(const bsp::BuzzerNoteDelayed* song) {
    if (buzzer_song == nullptr) {
        buzzer_song = song;
        osThreadFlagsSet(buzzerTaskHandle, BUZZER_SIGNAL);
        return true;
    }
    return false;
}
void buzzerTask(void* arg){
    UNUSED(arg);
    uint32_t flags = osThreadFlagsWait(BUZZER_SIGNAL, osFlagsWaitAll, osWaitForever);
        if (flags & BUZZER_SIGNAL) {
                if (buzzer_song != nullptr) {
                    buzzer->SingSong(buzzer_song, Buzzer_Delay);
                    buzzer_song = nullptr;
                }
        }
}


void init_buzzer() {
    buzzer = new bsp::Buzzer(&htim4, 3, 1000000);
}