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

// include this and enjoy automata
#include "Automata.h"

/*Global definitions for Host build, simulating input environment*/

// All states for the automata
enum class states { ON, OFF };
using states::ON, states::OFF;

// Raw data structures
struct raw_data_struct {
    int num1 = 0;
    bool dummy = false;
};

enum other_automata_output{s1, s2};

const int global_eight = 8;

int main() {
    raw_data_struct raw_data1;
    float num2 = 1.1;
    other_automata_output num3 = s1;

    /*
     * To build an automata, factory function's aid is necessary.
     * Class [AutomataBuilder] in charge of:
     * 1. Register correct type for input.
     * 2. Construct selected behaviour interface(component) for every input items.
     * 3. Define transition logic based on user definition.
     *
     * ---------------------------------[item]----------------------------------
     * The index of input items is arranged automatically start from 0 based on register order.
     *
     * Following types is supported to registered as an input in such grammar:
     * <type>                   <grammar>
     * ·Struct members          .item<...>(&Struct_name::member_name)
     * ·Common Variables        .item<...>(variable_name)
     * ·Enumerations            .item<...>(variable_name) or .item<...>(Enumeration_name::any_enum)
     *
     * Beside, you are always allow to do this:
     * .item<...>(Type{})
     *
     * Tip: Decompose a complex system by using small automatas' output as input.
     *
     * ---------------------------------[transition]----------------------------------
     * Symbols declare in global space can directly reference by transition logic.
     *
     * Transitions that define earlier have higher priority than later ones.
     * .transition<state_begin_with, state_goes_to>(TRANLOGIC { return a_bool; })
     *
     * Optional, it's allow to reverse logic aim for a neat grammar.
     * You can also explicitly clarify [ForwardTag] for better readability.
     * .transition<...>(TRANLOGIC {...}, [control::ForwardTag{}/control::ReverseTag{}])
     *
     * Using a registered Component by COMPONENT(registered_order) in transition logic definition.
     * Example: If there's .item<ComponentType>(Type{}) registered in 2nd place (index 1).
     *          COMPONENT(1) shall return [Component<Type> the_component]
     *
     * ---------------------------------[build]----------------------------------
     * It is not recommended to use function [build_heap_allocation] to build automata.
     * Extra time-space cost will occur using heap allocation.
     * Though it's an option to control life cycle manually.
     */
    auto aut = control::AutomataBuilder<states>()
        .item<control::AutomataInputEdge>(&raw_data_struct::num1)
        .item<control::AutomataInputEdge>(num2)
        .item<control::AutomataInputRaw>(num3)
        .item<control::AutomataInputRaw>(int32_t{})
        .transition<ON, OFF>(TRANLOGIC {
            auto& comp = COMPONENT(0);
            return comp.downEdge();
        })
        // u may insert comments freely like this
        // the transition below happens in a probability of 10%.
        .transition<OFF, ON>(TRANLOGIC { return rand() % 10 <= global_eight; }, control::ReverseTag{})
        .build<ON>();

    // You can also do this to assign to a global pointer:
    auto ptr = control::AutomataBuilder<states>()
        // ...
        .build_heap_allocation<ON>();
    ptr->input(std::make_tuple());

    // You may also define automatas base on same builder or in separate segmentations.
    // Though this will be verbose due to background implementation details ...
    auto builder_base =
        control::AutomataBuilder<states>().item<control::AutomataInputEdge>(num2);
    auto aut1_derive = builder_base
        .transition<ON, OFF>(TRANLOGIC { return true; })
        .build<ON>();
    auto aut2_derive = builder_base
        .transition<OFF, ON>(TRANLOGIC { return false; }, control::ReverseTag{})
        .build<OFF>();

    while (true) {
        // simulate value changes
        raw_data1.num1 = (++raw_data1.num1) % 10;
        num2 = (17.0 * num2 + 1.6) > 100.0 ?  (17.0 * num2 + 1.6) : 1;
        num3 = num3 == s1 ? s2 : s1;

        /*
         * Using a built automata is easy.
         * User only needs to take care input and output
         * multi-thread is also good (for now ;D)
         */

         /* input() gain update and drive automata to move a step based on these inputs.*/
        aut.input(std::make_tuple(raw_data1.num1, num2, num3, 3));

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