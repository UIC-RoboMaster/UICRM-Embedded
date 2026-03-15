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
// Created by gttgf on 2026/3/13.
//

#include "StateAutomatas.h"

namespace control {
    template <class EnumStatesCollection>
    StateAutomataBuilder<EnumStatesCollection>&
    StateAutomataBuilder<EnumStatesCollection>::transition(
        States from, Transition<EnumStatesCollection> trans) {
        state_machine_[static_cast<size_t>(from)].emplace_back(trans);
        return *this;
    }

    template <class EnumStatesCollection>
    StateAutomataBuilder<EnumStatesCollection>&
    StateAutomataBuilder<EnumStatesCollection>::transition(
        States from, States to, Predicate condition) {
        return transition(from, Transition<EnumStatesCollection>(condition, to));
    }

    /*Implemented in class declaration due to template grammar*/
    // template <class EnumStatesCollection>
    // template <template<class> class Component, typename Member, typename Struct>
    // StateAutomataBuilder<EnumStatesCollection>&
    // StateAutomataBuilder<EnumStatesCollection>::input(Member Struct::* member, const char* name) {
    // }

    template <class EnumStatesCollection>
    typename StateAutomataBuilder<EnumStatesCollection>::StateAutomata
    StateAutomataBuilder<EnumStatesCollection>::build(States init_state) {
        return StateAutomata(state_machine_, init_state, input_items_);
    }

    template <class EnumStatesCollection>
    template <typename... Ts>
    void StateAutomataBuilder<EnumStatesCollection>::StateAutomata::input(const std::tuple<Ts...>& data) {
        input_items_.updateItems(data);
        evaluateTransitions();
    }

    template <class EnumStatesCollection>
    typename StateAutomataBuilder<EnumStatesCollection>::States
    StateAutomataBuilder<EnumStatesCollection>::StateAutomata::state() {
        return current_state_;
    }

    //TODO implement demonstrate()
    //
    // template <class EnumStatesCollection>
    // void StateAutomataBuilder<EnumStatesCollection>::StateAutomata::demonstrate() {
    //
    // }

    template <class EnumStatesCollection>
    StateAutomataBuilder<EnumStatesCollection>::StateAutomata::StateAutomata(FSM machine, States init_state, Item inputs)
        : state_machine_(machine), current_state_(init_state), input_items_(inputs) {}

    template <class EnumStatesCollection>
    void StateAutomataBuilder<EnumStatesCollection>::StateAutomata::evaluateTransitions() {
        for (auto& it : state_machine_[current_state_])
            if (it.condition(input_items_)) { current_state_ = it.next; return; }
    }

}  // namespace control