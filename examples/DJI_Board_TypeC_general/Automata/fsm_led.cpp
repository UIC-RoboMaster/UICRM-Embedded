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

#include "cmsis_os.h"
#include "main.h"
#include "rgb.h"
#include "bsp_gpio.h"
#include "bsp_print.h"
#include "Automata.h"

enum key_states {RELEASE, PRESS};
control::AutomataBuilder<key_states>* key_builder;
control::Automata<key_states>* key_aut;

static display::RGB* led = nullptr;

void RM_RTOS_Init(void) {
    print_use_uart(&huart6, true, 921600);

    led = new display::RGB(&htim5, 3, 2, 1, 1000000);

    key_builder = new control::AutomataBuilder<key_states>();
    (*key_builder)
        .input<remote::AutomataInputRemote>(INTYPE(std::declval<bsp::GPIO>().Read()), "0")
        .transition(
            RELEASE,
            PRESS,
            TRANLOGIC {
                auto key_input =
                    ins.get<remote::AutomataInputRemote>(INTYPE(std::declval<bsp::GPIO>().Read()), 0);
                return key_input.get() == true;
            })
        .transition(
            PRESS,
            RELEASE,
            TRANLOGIC {
                auto key_input =
                    ins.get<remote::AutomataInputRemote>(INTYPE(std::declval<bsp::GPIO>().Read()), 0);
                return key_input.get() == false;
            });
    key_aut = key_builder->build(RELEASE);
    delete key_builder;

}

void RM_RTOS_Default_Task(const void* args) {
    UNUSED(args);

    bsp::GPIO key(KEY_GPIO_Port, KEY_Pin);

    int RGB_FLOW_COLOR_CHANGE_TIME = 300;

    uint32_t RGB_flow_color[] = {0xFFFF0000, 0xFF00FF00, 0xFF0000FF};
    const int size = sizeof(RGB_flow_color) / sizeof(RGB_flow_color[0]);

    float delta_alpha, delta_red, delta_green, delta_blue;
    float alpha, red, green, blue;
    uint32_t aRGB;

    int i = 0;
    while (true) {
        print_enabled("PRESSED", key_aut->state() ? PRESS : RELEASE);

        key_aut->input(std::make_tuple(key.Read()));
        if (key_aut->state() == RELEASE) continue;
        while (key_aut->state() == PRESS) {key_aut->input(std::make_tuple(key.Read()));}

        alpha = (RGB_flow_color[i] & 0xFF000000) >> 24;
        red = ((RGB_flow_color[i] & 0x00FF0000) >> 16);
        green = ((RGB_flow_color[i] & 0x0000FF00) >> 8);
        blue = ((RGB_flow_color[i] & 0x000000FF) >> 0);

        delta_alpha = (float)((RGB_flow_color[(i + 1) % 3] & 0xFF000000) >> 24) -
                      (float)((RGB_flow_color[i] & 0xFF000000) >> 24);
        delta_red = (float)((RGB_flow_color[(i + 1) % 3] & 0x00FF0000) >> 16) -
                    (float)((RGB_flow_color[i] & 0x00FF0000) >> 16);
        delta_green = (float)((RGB_flow_color[(i + 1) % 3] & 0x0000FF00) >> 8) -
                      (float)((RGB_flow_color[i] & 0x0000FF00) >> 8);
        delta_blue = (float)((RGB_flow_color[(i + 1) % 3] & 0x000000FF) >> 0) -
                     (float)((RGB_flow_color[i] & 0x000000FF) >> 0);

        delta_alpha /= RGB_FLOW_COLOR_CHANGE_TIME;
        delta_red /= RGB_FLOW_COLOR_CHANGE_TIME;
        delta_green /= RGB_FLOW_COLOR_CHANGE_TIME;
        delta_blue /= RGB_FLOW_COLOR_CHANGE_TIME;
        for (int j = 0; j < RGB_FLOW_COLOR_CHANGE_TIME; ++j) {
            alpha += delta_alpha;
            red += delta_red;
            green += delta_green;
            blue += delta_blue;

            aRGB = ((uint32_t)(alpha)) << 24 | ((uint32_t)(red)) << 16 | ((uint32_t)(green)) << 8 |
                   ((uint32_t)(blue)) << 0;
            led->Display(aRGB);
            osDelay(1);
        }

        ++i;
        i = i % size;
    }
}