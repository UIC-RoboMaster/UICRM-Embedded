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

#ifndef UICRM_STATEAUTOMATAS_H
#define UICRM_STATEAUTOMATAS_H

#include <functional>
#include <string>
#include <vector>
#include <variant>

#include "utils.h"

using std::vector;
using std::variant;

namespace control {

    /*InputTypeCollection*/
    /*
     TODO:
        Should abstract (EdgeDetector)s base class if remotes are all able to handle by basic types
        Keep std::variant to reach maximum extendability by adding new classes into this std::variant
        Or directly apply OOP abstraction to reach higher performance (doubt that :D)
    */
    using InputTypeCollection = std::variant<
        FloatEdgeDetector,
        BoolEdgeDetector
    >;
    typedef InputTypeCollection Input;
    /*InputTypeCollection*/

    /*Transition*/
    using Predicate = std::function<bool(const Input&)>;
    template <class EnumStatesCollection>
    struct Transition {
        Predicate condition;
        EnumStatesCollection next;
    };
    /*Transition*/

    /*StateAutomata*/
    template <class EnumStatesCollection>
    class StateAutomata {
    public:

        typedef EnumStatesCollection States;
        using FSM = vector<vector<Transition<States>>>;

        StateAutomata(FSM machine, States init_state);

        void step();

        std::string demonstrate() const;

    private:
        const FSM state_machine_;

        States current_state_;
    };
    /*StateAutomata*/

    /*StateAutomataBuilder*/
    template <class EnumStatesCollection>
    class StateAutomataBuilder {
    public:

        typedef EnumStatesCollection States;
        using FSM = vector<vector<Transition<States>>>;

        StateAutomataBuilder();

        StateAutomataBuilder& addState(States s);
        StateAutomataBuilder& addOutEdge(States from, Transition<States> trans);

        StateAutomata<EnumStatesCollection> build(States init_state);

    private:
        FSM state_machine_;
    };
    /*StateAutomataBuilder*/

}  // namespace control

#endif  // UICRM_STATEAUTOMATAS_H
