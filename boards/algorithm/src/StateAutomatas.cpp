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
    template <class EnumStatesCollection, class TupleData>
    StateAutomataBuilder<EnumStatesCollection, TupleData>&
    StateAutomataBuilder<EnumStatesCollection, TupleData>::transition(
        States from, Transition<States, TupleData> trans) {
        state_machine_[static_cast<size_t>(from)].emplace_back(trans);
        return *this;
    }

    template <class EnumStatesCollection, class TupleData>
    StateAutomataBuilder<EnumStatesCollection, TupleData>&
    StateAutomataBuilder<EnumStatesCollection, TupleData>::transition(
        States from, States to, Predicate cond) {
        return transition(from, Transition<EnumStatesCollection, TupleData>(cond, to));
    }

    template <class EnumStatesCollection, class TupleData>
    template <size_t Index, template <class> class Component>
    StateAutomataBuilder<EnumStatesCollection, TupleData>&
    StateAutomataBuilder<EnumStatesCollection, TupleData>::input(const char* name) {
        input_items_.template buildItem<Index, Component>(name);
        return *this;
    }

    template <class EnumStatesCollection, class TupleData>
    typename StateAutomataBuilder<EnumStatesCollection, TupleData>::StateAutomata
    StateAutomataBuilder<EnumStatesCollection, TupleData>::build(States init_state) {
        return StateAutomata(state_machine_, init_state, input_items_);
    }

    template <class EnumStatesCollection, class TupleData>
    void StateAutomataBuilder<EnumStatesCollection, TupleData>::StateAutomata::input(const TupleData& data) {
        input_items_.updateItems(data);
        evaluateTransitions();
    }

    template <class EnumStatesCollection, class TupleData>
    typename StateAutomataBuilder<EnumStatesCollection, TupleData>::States
    StateAutomataBuilder<EnumStatesCollection, TupleData>::StateAutomata::state() {
        return current_state_;
    }

    //TODO implement demonstrate()
    //
    // template <class EnumStatesCollection, class TupleData>
    // void StateAutomataBuilder<EnumStatesCollection, TupleData>::StateAutomata::demonstrate() {
    //
    // }

    template <class EnumStatesCollection, class TupleData>
    StateAutomataBuilder<EnumStatesCollection, TupleData>::StateAutomata::StateAutomata(FSM machine, States init_state, Item inputs)
        : state_machine_(machine), current_state_(init_state), input_items_(inputs) {}

    template <class EnumStatesCollection, class TupleData>
    void StateAutomataBuilder<EnumStatesCollection, TupleData>::StateAutomata::evaluateTransitions() {
        for (auto& it : state_machine_[current_state_])
            if (it.condition(input_items_)) { current_state_ = it.next; return; }
    }

}  // namespace control