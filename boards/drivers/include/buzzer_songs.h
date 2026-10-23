/*###########################################################
 # Copyright (c) 2026-2027. BNU-HKBU UIC RoboMaster         #
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

namespace driver {
    namespace songs {

        // Shared melodies and status tones. All delays are in milliseconds.
        // End each sequence with Finish so Buzzer::SingSong stops at its boundary.
        inline constexpr BuzzerNoteDelayed Mario[] =
            {{BuzzerNote::Mi3M, 80},
             {BuzzerNote::Silent, 80},
             {BuzzerNote::Mi3M, 80},
             {BuzzerNote::Silent, 240},
             {BuzzerNote::Mi3M, 80},
             {BuzzerNote::Silent, 240},
             {BuzzerNote::Do1M, 80},
             {BuzzerNote::Silent, 80},
             {BuzzerNote::Mi3M, 80},
             {BuzzerNote::Silent, 240},
             {BuzzerNote::So5M, 80},
             {BuzzerNote::Silent, 560},
             {BuzzerNote::So5L, 80},
             {BuzzerNote::Silent, 0},
             {BuzzerNote::Finish, 0}};

        inline constexpr BuzzerNoteDelayed War_Cant_of_Mars[] = {
            {BuzzerNote::So5M, 400},   {BuzzerNote::So5M, 200},   {BuzzerNote::So5M, 200},   {BuzzerNote::So5M, 400},
            {BuzzerNote::Fa4M, 200},   {BuzzerNote::Mi3M, 400},   {BuzzerNote::So5M, 200},   {BuzzerNote::Do1H, 400},
            {BuzzerNote::Re2H, 200},   {BuzzerNote::Mi3H, 400},   {BuzzerNote::Mi3H, 200},   {BuzzerNote::Mi3H, 400},
            {BuzzerNote::Re2H, 200},   {BuzzerNote::Do1H, 400},   {BuzzerNote::Do1H, 400},   {BuzzerNote::Si7M, 200},
            {BuzzerNote::La6M, 400},   {BuzzerNote::La6M, 200},   {BuzzerNote::La6M, 400},   {BuzzerNote::Si7M, 200},
            {BuzzerNote::Do1H, 400},   {BuzzerNote::Si7M, 200},   {BuzzerNote::Do1H, 400},   {BuzzerNote::La6M, 200},
            {BuzzerNote::So5M, 400},   {BuzzerNote::La6M, 200},   {BuzzerNote::So5M, 400},   {BuzzerNote::Mi3M, 200},
            {BuzzerNote::So5M, 800},   {BuzzerNote::So5M, 400},   {BuzzerNote::So5M, 200},   {BuzzerNote::So5M, 400},
            {BuzzerNote::So5M, 200},   {BuzzerNote::So5M, 400},   {BuzzerNote::Fa4M, 200},   {BuzzerNote::Mi3M, 400},
            {BuzzerNote::So5M, 200},   {BuzzerNote::Do1H, 400},   {BuzzerNote::Re2H, 200},   {BuzzerNote::Mi3H, 400},
            {BuzzerNote::Mi3H, 200},   {BuzzerNote::Mi3H, 400},   {BuzzerNote::Re2H, 200},   {BuzzerNote::Do1H, 800},
            {BuzzerNote::Do1H, 800},   {BuzzerNote::Re2H, 800},   {BuzzerNote::Re2H, 800},   {BuzzerNote::Do1H, 800},
            {BuzzerNote::Si7M, 800},   {BuzzerNote::Do1H, 1600},  {BuzzerNote::Silent, 400}, {BuzzerNote::Silent, 400},
            {BuzzerNote::So5M, 800},   {BuzzerNote::Fa4M, 400},   {BuzzerNote::Mi3M, 400},   {BuzzerNote::So5M, 200},
            {BuzzerNote::Do1H, 400},   {BuzzerNote::Re2H, 200},   {BuzzerNote::Mi3H, 1200},  {BuzzerNote::Do1H, 800},
            {BuzzerNote::Silent, 400}, {BuzzerNote::La6M, 800},   {BuzzerNote::Si7M, 400},   {BuzzerNote::Do1H, 400},
            {BuzzerNote::Si7M, 200},   {BuzzerNote::Do1H, 400},   {BuzzerNote::La6M, 200},   {BuzzerNote::So5M, 1600},
            {BuzzerNote::Mi3M, 800},   {BuzzerNote::Silent, 400}, {BuzzerNote::So5M, 800},   {BuzzerNote::Fa4M, 400},
            {BuzzerNote::Mi3M, 400},   {BuzzerNote::So5M, 200},   {BuzzerNote::Do1H, 400},   {BuzzerNote::Re2H, 200},
            {BuzzerNote::Mi3H, 1600},  {BuzzerNote::Do1H, 800},   {BuzzerNote::Do1H, 800},   {BuzzerNote::Re2H, 800},
            {BuzzerNote::Re2H, 800},   {BuzzerNote::Do1H, 800},   {BuzzerNote::Si7M, 800},   {BuzzerNote::Do1H, 1600},
            {BuzzerNote::Silent, 0},   {BuzzerNote::Finish, 0},
        };

        inline constexpr BuzzerNoteDelayed DJI[] =
            {{BuzzerNote::Do1M, 300}, {BuzzerNote::Re2M, 300}, {BuzzerNote::So5M, 600}, {BuzzerNote::Finish, 0}};

        inline constexpr BuzzerNoteDelayed SingCaliStart[] =
            {{BuzzerNote::Mi3M, 100}, {BuzzerNote::Re2M, 100}, {BuzzerNote::Mi3M, 100}, {BuzzerNote::Finish, 0}};

        inline constexpr BuzzerNoteDelayed SingCaliDone[] =
            {{BuzzerNote::Re2M, 100}, {BuzzerNote::Mi3M, 100}, {BuzzerNote::Re2M, 100}, {BuzzerNote::Finish, 0}};

        // Two Tigers.
        inline constexpr BuzzerNoteDelayed Laohu[] =
            {{BuzzerNote::Do1M, 500},  {BuzzerNote::Re2M, 500},   {BuzzerNote::Mi3M, 500}, {BuzzerNote::Do1M, 450},
             {BuzzerNote::Silent, 50}, {BuzzerNote::Do1M, 500},   {BuzzerNote::Re2M, 500}, {BuzzerNote::Mi3M, 500},
             {BuzzerNote::Do1M, 450},  {BuzzerNote::Silent, 50},  {BuzzerNote::Mi3M, 500}, {BuzzerNote::Fa4M, 500},
             {BuzzerNote::So5M, 500},  {BuzzerNote::Silent, 500}, {BuzzerNote::Mi3M, 500}, {BuzzerNote::Fa4M, 500},
             {BuzzerNote::So5M, 500},  {BuzzerNote::Silent, 500}, {BuzzerNote::So5M, 250}, {BuzzerNote::La6M, 250},
             {BuzzerNote::So5M, 250},  {BuzzerNote::Fa4M, 250},   {BuzzerNote::Mi3M, 500}, {BuzzerNote::Do1M, 500},
             {BuzzerNote::So5M, 250},  {BuzzerNote::La6M, 250},   {BuzzerNote::So5M, 250}, {BuzzerNote::Fa4M, 250},
             {BuzzerNote::Mi3M, 500},  {BuzzerNote::Do1M, 500},   {BuzzerNote::Re2M, 500}, {BuzzerNote::Si7L, 500},
             {BuzzerNote::Do1M, 500},  {BuzzerNote::Silent, 500}, {BuzzerNote::Re2M, 500}, {BuzzerNote::Si7L, 500},
             {BuzzerNote::Do1M, 500},  {BuzzerNote::Silent, 500}, {BuzzerNote::Finish, 0}};

        // Never Gonna Give You Up - Rick Astley.
        // 114 BPM; delays in milliseconds. Each pitched note sounds for 90% of its slot.
        inline constexpr BuzzerNoteDelayed NeverGonnaGiveYouUp[] = {
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
