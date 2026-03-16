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

// =====================
// Global definitions for Host build, simulating input environment
// =====================

// All states for the automata
enum states { ON, OFF };

// Raw data structures
struct raw_data_structA {
    int num1 = 0;
    bool dummy = false;
};
struct raw_data_structB {
    float num2 = 0.0;
    bool dummy = true;
};

int main() {
    // using namespace hostfsm;
    raw_data_structA raw_data1;
    raw_data_structB raw_data2;

    /*
     * To build an automata, factory class's aid is necessary.
     * Class [AutomataBuilder] in charge of:
     * 1. construct correct type for input,
     * 2. construct correct behaviour interface(component) for every input items, and
     * 3. construct transition logic based on user definition.
     *
     * The reason here using dynamic allocation to create object is
     * user might want to control life cycle of factory class manually.
     * builder will temporarily restore construction info in it.
     * If NO other forward automata needs to be constructed, builder is RECOMMENDED to be deleted.
     * (Or use smart pointers [unique_ptr] to do so. Strict domain design required.)
     */
    auto* builder = new control::AutomataBuilder<states>();
    //using can improve code readability, though be careful.
    using communication::Ins;
    using remote::AutomataInputRemote;

    /*
     * This's where to define all transition logic and dependent data item in automata
     * There are only 2 types of definition:
     *
     * input: define what to input including data type(by passing struct segment)
     * and behaviour(by appoint [AutomataInput] class type)
     *
     * transiton: define transition logic including the origin state and target state,
     * and transition logic in a form of lambda function
     *
     * The design of builder aim to not let user concern of type of input item(variable).
     * You may feel free to register different types items in automata.
     * As long as use tuple as input type later when you are using automata.
     *
     * TRANLOGIC is a fixed form to aid logic construction.
     * It is actually a lambda function head.
     * It is free to use an none-registered variant in lambda function as long as in this domain.
     * lambda function should as small as possible to
     * prevent extra performance cost(heap allocation).
     * DO NOT use complex functional programming here!(recursive, partial application, nested etc.)
     */
    (*builder)
        .input<AutomataInputRemote>(&raw_data_structA::num1, "1st") // index 1
        .input<AutomataInputRemote>(&raw_data_structB::num2, "2nd") // index 2
        .transition(
            ON, OFF,
            TRANLOGIC { //TODO use custom name instead of index
                auto& comp1 = ins.get<AutomataInputRemote>(0, &raw_data_structA::num1);
                return comp1.downEdge();
            })
        .transition(
            OFF, ON, TRANLOGIC { return rand() % 1000 > 990; });

    /*
     * The step that construct an actual automata
     * Automata shall not change under any case after being built.
     * (I hope my code fully prevent it :D)
     * (This is also the principle why all these type matic may happen.)
     */
    control::Automata<states>* aut = builder->build(OFF);
    delete builder;  // builder holds duplicated info, recommend to delete

    while (true) {
        // simulate value changes
        raw_data1.num1 = (++raw_data1.num1) % 50;
        raw_data2.num2 = static_cast<int>((raw_data2.num2 * 6 + 1)) % 100;

        /*
         * Using a built automata is easy.
         * User only needs to care input and output
         * multi-thread is also good (for now ;D)
         */

         /* input() gain update and drive automata to move a step based on these inputs.*/
        aut->input(std::make_tuple(raw_data1.num1, raw_data2.num2));

        std::cout << "num1:" << raw_data1.num1 << " num2:" << raw_data2.num2 << " " << std::endl;
        std::cout << "state:";
        /* And here's how to get current automata output*/
        switch (aut->state()) {
            case ON:
                std::cout << "ON" << std::endl;
                break;
            case OFF:
                std::cout << "OFF" << std::endl;
                break;
        }
        std::cout << std::endl;
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
}