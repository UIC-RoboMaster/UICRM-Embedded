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
// Global definitions for Host build
// =====================
// namespace hostfsm {

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

// } // namespace hostfsm

int main() {
    // using namespace hostfsm;
    raw_data_structA raw_data1;
    raw_data_structB raw_data2;

    auto* builder = new control::AutomataBuilder<states>();
    using communication::Ins;
    using remote::AutomataInputRemote;

    // =====================
    // Lambda predicates explicitly cast to std::function
    // =====================
    std::function<bool(const Ins&)> pred_off = [](const Ins& ins) -> bool {
        auto comp1 = ins.get<AutomataInputRemote>(0, &raw_data_structA::num1);
        auto comp2 = ins.get<AutomataInputRemote>(1, &raw_data_structB::num2);
        // return comp1.get() > 100000 && comp2.edge();
        return comp1.downEdge();
    };

    std::function<bool(const Ins&)> pred_on = [](const Ins&) -> bool {
        return rand() % 1000 >= 990;
    };

    (*builder)
        .input<AutomataInputRemote>(&raw_data_structA::num1, "1st")
        .input<AutomataInputRemote>(&raw_data_structB::num2, "2nd")
        .transition(ON, OFF, pred_off)
        .transition(OFF, ON, pred_on);

    control::Automata<states>* aut = builder->build(OFF);
    delete builder;  // builder holds duplicated info, safe to delete

    while (true) {
        raw_data1.num1 = (++raw_data1.num1) % 50;
        raw_data2.num2 = static_cast<int>((++raw_data2.num2)) % 50000;

        aut->input(std::make_tuple(raw_data1.num1, raw_data2.num2));

        std::cout << raw_data1.num1 << " " << raw_data2.num2 << " ";
        switch (aut->state()) {
            case ON:
                std::cout << "ON" << std::endl;
                break;
            case OFF:
                std::cout << "OFF" << std::endl;
                break;
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(200));
    }
}