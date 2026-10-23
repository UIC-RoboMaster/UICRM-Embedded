#pragma once

#include "buzzer.h"

namespace driver {
    namespace songs {

        // Never Gonna Give You Up - Rick Astley.
        // 114 BPM; delays in milliseconds. Each pitched note sounds for 90% of its slot.
        static constexpr BuzzerNoteDelayed NeverGonnaGiveYouUp[] = {
            // Intro
            {BuzzerNote::Re2M, 710},
            {BuzzerNote::Silent, 79},  // D5, -4
            {BuzzerNote::Mi3M, 710},
            {BuzzerNote::Silent, 79},  // E5, -4
            {BuzzerNote::La6L, 473},
            {BuzzerNote::Silent, 53},  // A4, 4

            {BuzzerNote::Mi3M, 710},
            {BuzzerNote::Silent, 79},  // E5, -4
            {BuzzerNote::Fa4SharpM, 710},
            {BuzzerNote::Silent, 79},  // FS5, -4
            {BuzzerNote::La6M, 117},
            {BuzzerNote::Silent, 14},  // A5, 16
            {BuzzerNote::So5M, 117},
            {BuzzerNote::Silent, 14},  // G5, 16
            {BuzzerNote::Fa4SharpM, 236},
            {BuzzerNote::Silent, 27},  // FS5, 8

            {BuzzerNote::Re2M, 710},
            {BuzzerNote::Silent, 79},  // D5, -4
            {BuzzerNote::Mi3M, 710},
            {BuzzerNote::Silent, 79},  // E5, -4
            {BuzzerNote::La6L, 946},
            {BuzzerNote::Silent, 106},  // A4, 2

            {BuzzerNote::La6L, 117},
            {BuzzerNote::Silent, 14},  // A4, 16
            {BuzzerNote::La6L, 117},
            {BuzzerNote::Silent, 14},  // A4, 16
            {BuzzerNote::Si7L, 117},
            {BuzzerNote::Silent, 14},  // B4, 16
            {BuzzerNote::Re2M, 236},
            {BuzzerNote::Silent, 27},  // D5, 8
            {BuzzerNote::Re2M, 117},
            {BuzzerNote::Silent, 14},  // D5, 16

            {BuzzerNote::Re2M, 710},
            {BuzzerNote::Silent, 79},  // D5, -4
            {BuzzerNote::Mi3M, 710},
            {BuzzerNote::Silent, 79},  // E5, -4
            {BuzzerNote::La6L, 473},
            {BuzzerNote::Silent, 53},  // A4, 4

            {BuzzerNote::Mi3M, 710},
            {BuzzerNote::Silent, 79},  // E5, -4
            {BuzzerNote::Fa4SharpM, 710},
            {BuzzerNote::Silent, 79},  // FS5, -4
            {BuzzerNote::La6M, 117},
            {BuzzerNote::Silent, 14},  // A5, 16
            {BuzzerNote::So5M, 117},
            {BuzzerNote::Silent, 14},  // G5, 16
            {BuzzerNote::Fa4SharpM, 236},
            {BuzzerNote::Silent, 27},  // FS5, 8

            {BuzzerNote::Re2M, 710},
            {BuzzerNote::Silent, 79},  // D5, -4
            {BuzzerNote::Mi3M, 710},
            {BuzzerNote::Silent, 79},  // E5, -4
            {BuzzerNote::La6L, 946},
            {BuzzerNote::Silent, 106},  // A4, 2

            // Chorus
            {BuzzerNote::La6L, 117},
            {BuzzerNote::Silent, 14},  // A4, 16
            {BuzzerNote::Si7L, 117},
            {BuzzerNote::Silent, 14},  // B4, 16
            {BuzzerNote::Re2M, 117},
            {BuzzerNote::Silent, 14},  // D5, 16
            {BuzzerNote::Si7L, 117},
            {BuzzerNote::Silent, 14},  // B4, 16

            {BuzzerNote::Fa4SharpM, 354},
            {BuzzerNote::Silent, 40},  // FS5, -8
            {BuzzerNote::Fa4SharpM, 354},
            {BuzzerNote::Silent, 40},  // FS5, -8
            {BuzzerNote::Mi3M, 710},
            {BuzzerNote::Silent, 79},  // E5, -4
            {BuzzerNote::La6L, 117},
            {BuzzerNote::Silent, 14},  // A4, 16
            {BuzzerNote::Si7L, 117},
            {BuzzerNote::Silent, 14},  // B4, 16
            {BuzzerNote::Re2M, 117},
            {BuzzerNote::Silent, 14},  // D5, 16
            {BuzzerNote::Si7L, 117},
            {BuzzerNote::Silent, 14},  // B4, 16

            {BuzzerNote::Mi3M, 354},
            {BuzzerNote::Silent, 40},  // E5, -8
            {BuzzerNote::Mi3M, 354},
            {BuzzerNote::Silent, 40},  // E5, -8
            {BuzzerNote::Re2M, 354},
            {BuzzerNote::Silent, 40},  // D5, -8
            {BuzzerNote::Do1SharpM, 117},
            {BuzzerNote::Silent, 14},  // CS5, 16
            {BuzzerNote::Si7L, 354},
            {BuzzerNote::Silent, 40},  // B4, -8
            {BuzzerNote::La6L, 117},
            {BuzzerNote::Silent, 14},  // A4, 16
            {BuzzerNote::Si7L, 117},
            {BuzzerNote::Silent, 14},  // B4, 16
            {BuzzerNote::Re2M, 117},
            {BuzzerNote::Silent, 14},  // D5, 16
            {BuzzerNote::Si7L, 117},
            {BuzzerNote::Silent, 14},  // B4, 16

            {BuzzerNote::Re2M, 473},
            {BuzzerNote::Silent, 53},  // D5, 4
            {BuzzerNote::Mi3M, 236},
            {BuzzerNote::Silent, 27},  // E5, 8
            {BuzzerNote::Do1SharpM, 354},
            {BuzzerNote::Silent, 40},  // CS5, -8
            {BuzzerNote::Si7L, 117},
            {BuzzerNote::Silent, 14},  // B4, 16
            {BuzzerNote::La6L, 236},
            {BuzzerNote::Silent, 27},  // A4, 8
            {BuzzerNote::La6L, 236},
            {BuzzerNote::Silent, 27},  // A4, 8
            {BuzzerNote::La6L, 236},
            {BuzzerNote::Silent, 27},  // A4, 8

            {BuzzerNote::Mi3M, 473},
            {BuzzerNote::Silent, 53},  // E5, 4
            {BuzzerNote::Re2M, 946},
            {BuzzerNote::Silent, 106},  // D5, 2
            {BuzzerNote::La6L, 117},
            {BuzzerNote::Silent, 14},  // A4, 16
            {BuzzerNote::Si7L, 117},
            {BuzzerNote::Silent, 14},  // B4, 16
            {BuzzerNote::Re2M, 117},
            {BuzzerNote::Silent, 14},  // D5, 16
            {BuzzerNote::Si7L, 117},
            {BuzzerNote::Silent, 14},  // B4, 16

            {BuzzerNote::Fa4SharpM, 354},
            {BuzzerNote::Silent, 40},  // FS5, -8
            {BuzzerNote::Fa4SharpM, 354},
            {BuzzerNote::Silent, 40},  // FS5, -8
            {BuzzerNote::Mi3M, 710},
            {BuzzerNote::Silent, 79},  // E5, -4
            {BuzzerNote::La6L, 117},
            {BuzzerNote::Silent, 14},  // A4, 16
            {BuzzerNote::Si7L, 117},
            {BuzzerNote::Silent, 14},  // B4, 16
            {BuzzerNote::Re2M, 117},
            {BuzzerNote::Silent, 14},  // D5, 16
            {BuzzerNote::Si7L, 117},
            {BuzzerNote::Silent, 14},  // B4, 16

            {BuzzerNote::La6M, 473},
            {BuzzerNote::Silent, 53},  // A5, 4
            {BuzzerNote::Do1SharpM, 236},
            {BuzzerNote::Silent, 27},  // CS5, 8
            {BuzzerNote::Re2M, 354},
            {BuzzerNote::Silent, 40},  // D5, -8
            {BuzzerNote::Do1SharpM, 117},
            {BuzzerNote::Silent, 14},  // CS5, 16
            {BuzzerNote::Si7L, 236},
            {BuzzerNote::Silent, 27},  // B4, 8
            {BuzzerNote::La6L, 117},
            {BuzzerNote::Silent, 14},  // A4, 16
            {BuzzerNote::Si7L, 117},
            {BuzzerNote::Silent, 14},  // B4, 16
            {BuzzerNote::Re2M, 117},
            {BuzzerNote::Silent, 14},  // D5, 16
            {BuzzerNote::Si7L, 117},
            {BuzzerNote::Silent, 14},  // B4, 16

            {BuzzerNote::Re2M, 473},
            {BuzzerNote::Silent, 53},  // D5, 4
            {BuzzerNote::Mi3M, 236},
            {BuzzerNote::Silent, 27},  // E5, 8
            {BuzzerNote::Do1SharpM, 354},
            {BuzzerNote::Silent, 40},  // CS5, -8
            {BuzzerNote::Si7L, 117},
            {BuzzerNote::Silent, 14},  // B4, 16
            {BuzzerNote::La6L, 473},
            {BuzzerNote::Silent, 53},  // A4, 4
            {BuzzerNote::La6L, 236},
            {BuzzerNote::Silent, 27},  // A4, 8

            {BuzzerNote::Mi3M, 473},
            {BuzzerNote::Silent, 53},  // E5, 4
            {BuzzerNote::Re2M, 946},
            {BuzzerNote::Silent, 106},  // D5, 2
            {BuzzerNote::Silent, 526},  // REST, 4

            {BuzzerNote::Finish, 0},
        };

    }  // namespace songs
}  // namespace driver
