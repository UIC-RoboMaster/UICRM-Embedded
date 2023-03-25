#include "bsp_buzzer.h"

namespace bsp {

    Buzzer::Buzzer(TIM_HandleTypeDef* htim, uint32_t channel, uint32_t clock_freq)
        : pwm_(htim, channel, clock_freq, 0, 0) {
        pwm_.Start();
    }

    void Buzzer::SingTone(const BuzzerNote& note) {
        if (note != BuzzerNote::Finish) {
            pwm_.SetFrequency(static_cast<uint32_t>(note));
            pwm_.SetPulseWidth(1000000 / static_cast<uint32_t>(note) / 2);
        }
    }

    void Buzzer::SingSong(const BuzzerNoteDelayed* delayed_notes, buzzer_delay_t delay_func) {
        while (delayed_notes->note != BuzzerNote::Finish) {
            SingTone(delayed_notes->note);
            delay_func(delayed_notes->delay);
            ++delayed_notes;
        }
        Off();
    }

    void Buzzer::Off() {
        pwm_.SetPulseWidth(0);
    }

} /* namespace bsp */
