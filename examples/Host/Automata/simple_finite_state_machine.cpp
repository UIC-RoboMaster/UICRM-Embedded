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

#include <cstdlib>
#include <functional>
#include <iostream>
#include <thread>
#include <tuple>

#include "Automata.h"

/*Global definitions for Host build, simulating input environment*/

// All states for the automata
enum states { ON, OFF };

// Raw data structures
struct raw_data_struct {
    int num1 = 0;
    bool dummy = false;
};

enum other_automata_output{s1, s2};

int main() {
    raw_data_struct raw_data1;
    float num2 = 1.1;
    other_automata_output num3 = s1;

    /*
     * To build an automata, factory function's aid is necessary.
     * Static function [make_automata] in charge of:
     * 1. construct correct type for input,
     * 2. construct correct behaviour interface(component) for every input items, and
     * 3. construct transition logic based on user definition.
     *
     * It is important that the index of input items is arranged automatically start from 0.
     *
     * Transitions that define earlier have higher priority than later ones.
     *
     * It is recommended to use member function [build] to "make automata".
     * Though there's actually another option [build_unique]
     * in order to perform heap allocation.
     *
     * The reason here giving an option of [build_unique] to create object is
     * user might want to control life cycle of factory class manually.
     */
    auto aut = control::AutomataBuilder<states>()
        .item<control::AutomataInputRemote>(&raw_data_struct::num1)
        .item<control::AutomataInputRemote>(num2)
        .item<control::AutomataInputRaw>(num3)
        .transition<ON, OFF>(TRANLOGIC { return ins.template get<0>().get()==2; })
        .transition<OFF, ON>(TRANLOGIC { return rand() % 10 > 8; })
        .build<ON>();

    /*
     * You can also:
     * auto aut_ptr = make_automata_unique(...);
     */

    while (true) {
        // simulate value changes
        raw_data1.num1 = (++raw_data1.num1) % 10;
        num2 = (17.0 * num2 + 1.6) > 100.0 ?  (17.0 * num2 + 1.6) : 1;
        num3 = num3 == s1 ? s2 : s1;

        /*
         * Using a built automata is easy.
         * User only needs to care input and output
         * multi-thread is also good (for now ;D)
         */

         /* input() gain update and drive automata to move a step based on these inputs.*/
        aut.input(std::make_tuple(raw_data1.num1, num2, num3));

        std::cout << "num1:" << raw_data1.num1 <<
            " num2:" << num2 <<
            " num3:" << (num3 == s1 ? "s1" : "s2") << std::endl;

        std::cout << "state:";
        /* And here's how to get current automata output*/
        switch (aut.state()) {
            case ON:
                std::cout << "ON" << std::endl;
                break;
            case OFF:
                std::cout << "OFF" << std::endl;
                break;
        }
        std::cout << std::endl;
        std::this_thread::sleep_for(std::chrono::milliseconds(200));
    }
}