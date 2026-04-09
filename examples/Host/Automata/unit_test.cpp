// Copyright (c) 2026. BNU-HKBU UIC RoboMaster
//
// This program is free software: you can redistribute it
// and/or modify it under the terms of the GNU General
// Public License as published by the Free Software
// Foundation, either version 3 of the License, or (at
// your option) any later version.
//
// This program is distributed in the hope that it will be
// useful, but WITHOUT ANY WARRANTY; without even
// the implied warranty of MERCHANTABILITY or FITNESS
// FOR A PARTICULAR PURPOSE.  See the GNU General
// Public License for more details.
//
// You should have received a copy of the GNU General
// Public License along with this program.  If not, see
// <https://www.gnu.org/licenses/>.

//
// Created by gttgf on 2026/3/19.
//

#include <iostream>
#include <ostream>
#include <thread>

#include "Automata.h"
#include "Automata/AutomataInputManagement.h"

enum States {s1, s2};

int main() {

    auto inputs = control::CollectItems()
        .addItem<control::AutomataInputEdge>(int{})
        .output();

    auto fsm = control::CollectTransitions<States>()
        .addTrans<s1, s2, control::ForwardTag>([](const auto& ins) -> bool { return ins.template get<0>().get()==2; })
        .addTrans<s2, s1, control::ForwardTag>([](const auto& ins) -> bool { return true; })
        .output(s1);

    int num = 0;
    while (true) {

        num = (num + 1) % 3;
        inputs.updateItems(std::make_tuple(num));

        fsm.step(inputs);
        switch (fsm.state()) {
            case s1:
                std::cout << "s1" << std::endl;
                break;
            case s2:
                std::cout << "s2" << std::endl;
                break;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
    }
}
