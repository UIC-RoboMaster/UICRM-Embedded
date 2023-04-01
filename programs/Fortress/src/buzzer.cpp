#include "buzzer.h"

bsp::Buzzer* buzzer = nullptr;

void init_buzzer() {
    buzzer = new bsp::Buzzer(&htim4, 3, 1000000);
}