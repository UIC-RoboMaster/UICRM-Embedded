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

#include "main.h"

#include "buzzer.h"
#include "tim.h"

using Note = driver::BuzzerNote;

// static bsp::BuzzerNoteDelayed Mario[] = {
//     {Note::Mi3M, 80}, {Note::Silent, 80},  {Note::Mi3M, 80}, {Note::Silent, 240},
//     {Note::Mi3M, 80}, {Note::Silent, 240}, {Note::Do1M, 80}, {Note::Silent, 80},
//     {Note::Mi3M, 80}, {Note::Silent, 240}, {Note::So5M, 80}, {Note::Silent, 560},
//     {Note::So5L, 80}, {Note::Silent, 0},   {Note::Finish, 0}};
//  uncomment to play the full song
// static bsp::BuzzerNoteDelayed War_Cant_of_Mars[] = {
//     {Note::So5M, 400},   {Note::So5M, 200},   {Note::So5M, 200},   {Note::So5M, 400},
//     {Note::Fa4M, 200},   {Note::Mi3M, 400},   {Note::So5M, 200},   {Note::Do1H, 400},
//     {Note::Re2H, 200},   {Note::Mi3H, 400},   {Note::Mi3H, 200},   {Note::Mi3H, 400},
//     {Note::Re2H, 200},   {Note::Do1H, 400},   {Note::Do1H, 400},   {Note::Si7M, 200},
//     {Note::La6M, 400},   {Note::La6M, 200},   {Note::La6M, 400},   {Note::Si7M, 200},
//     {Note::Do1H, 400},   {Note::Si7M, 200},   {Note::Do1H, 400},   {Note::La6M, 200},
//     {Note::So5M, 400},   {Note::La6M, 200},   {Note::So5M, 400},   {Note::Mi3M, 200},
//     {Note::So5M, 800},   {Note::So5M, 400},   {Note::So5M, 200},   {Note::So5M, 400},
//     {Note::So5M, 200},   {Note::So5M, 400},   {Note::Fa4M, 200},   {Note::Mi3M, 400},
//     {Note::So5M, 200},   {Note::Do1H, 400},   {Note::Re2H, 200},   {Note::Mi3H, 400},
//     {Note::Mi3H, 200},   {Note::Mi3H, 400},   {Note::Re2H, 200},   {Note::Do1H, 800},
//     {Note::Do1H, 800},   {Note::Re2H, 800},   {Note::Re2H, 800},   {Note::Do1H, 800},
//     {Note::Si7M, 800},   {Note::Do1H, 1600},  {Note::Silent, 400}, {Note::Silent, 400},
//     {Note::So5M, 800},   {Note::Fa4M, 400},   {Note::Mi3M, 400},   {Note::So5M, 200},
//     {Note::Do1H, 400},   {Note::Re2H, 200},   {Note::Mi3H, 1200},  {Note::Do1H, 800},
//     {Note::Silent, 400}, {Note::La6M, 800},   {Note::Si7M, 400},   {Note::Do1H, 400},
//     {Note::Si7M, 200},   {Note::Do1H, 400},   {Note::La6M, 200},   {Note::So5M, 1600},
//     {Note::Mi3M, 800},   {Note::Silent, 400}, {Note::So5M, 800},   {Note::Fa4M, 400},
//     {Note::Mi3M, 400},   {Note::So5M, 200},   {Note::Do1H, 400},   {Note::Re2H, 200},
//     {Note::Mi3H, 1600},  {Note::Do1H, 800},   {Note::Do1H, 800},   {Note::Re2H, 800},
//     {Note::Re2H, 800},   {Note::Do1H, 800},   {Note::Si7M, 800},   {Note::Do1H, 1600},
//     {Note::Silent, 0},   {Note::Finish, 0},
// };

static driver::BuzzerNoteDelayed DJI[] = {
    {Note::Do1M, 300}, {Note::Re2M, 300}, {Note::So5M, 600}, {Note::Finish, 0}};

// static driver::BuzzerNoteDelayed Laohu[] = {
//     {Note::Do1M, 500},  {Note::Re2M, 500},   {Note::Mi3M, 500}, {Note::Do1M, 450},
//     {Note::Silent, 50}, {Note::Do1M, 500},   {Note::Re2M, 500}, {Note::Mi3M, 500},
//     {Note::Do1M, 450},  {Note::Silent, 50},  {Note::Mi3M, 500}, {Note::Fa4M, 500},
//     {Note::So5M, 500},  {Note::Silent, 500}, {Note::Mi3M, 500}, {Note::Fa4M, 500},
//     {Note::So5M, 500},  {Note::Silent, 500}, {Note::So5M, 250}, {Note::La6M, 250},
//     {Note::So5M, 250},  {Note::Fa4M, 250},   {Note::Mi3M, 500}, {Note::Do1M, 500},
//     {Note::So5M, 250},  {Note::La6M, 250},   {Note::So5M, 250}, {Note::Fa4M, 250},
//     {Note::Mi3M, 500},  {Note::Do1M, 500},   {Note::Re2M, 500}, {Note::Si7L, 500},
//     {Note::Do1M, 500},  {Note::Silent, 500}, {Note::Re2M, 500}, {Note::Si7L, 500},
//     {Note::Do1M, 500},  {Note::Silent, 500}, {Note::Finish, 0}};

void RM_RTOS_Init(void) {
    driver::Buzzer buzzer(&htim12, 2, 1000000);
    buzzer.SingSong(DJI);
    HAL_Delay(500);
    //    buzzer.SingSong(Laohu);
    HAL_Delay(1000);
}
