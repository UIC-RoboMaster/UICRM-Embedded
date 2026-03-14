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

using std::vector;
using std::variant;

namespace communication
{
    template<class TupleData>
    class AutomataInputManagement;
}
using communication::AutomataInputManagement;

namespace control {

    /*Transition*/

    template <class EnumStatesCollection, class TupleData>
    struct Transition {

        using Predicate = std::function<bool(const AutomataInputManagement<TupleData>&)>;

        Predicate condition;
        EnumStatesCollection next;
    };
    /*Transition*/

    /*StateAutomata*/
    /**
     * An automata that work based on graph.
     *
     * Transitions are depend on a condition that in a form of std::function(function pointer) which its return value always boolean.
     *
     * This class should be constructed by class [control::StateAutomataBuilder].
     * It's NOT recommended that user construct Finite State Machine(FSM) without the aid of factory class.
     *
     * @tparam EnumStatesCollection Enumeration of all states that expected to perform in the automata.
     * @tparam TupleData Tuple form data package.
     */
    template <class EnumStatesCollection, class TupleData>
    class StateAutomata {
    public:

        typedef EnumStatesCollection States;
        using FSM = vector<vector<Transition<States, TupleData>>>;
        using Item = AutomataInputManagement<TupleData>;

        StateAutomata(FSM machine, States init_state, Item inputs);

        void input(const TupleData&);
        States state();

        std::string demonstrate() const;

    private:
        const FSM state_machine_;
        States current_state_;

        Item input_items_;

        void evaluateTransitions();
    };
    /*StateAutomata*/

    /*StateAutomataBuilder*/
    /**
     * Factory class that guarantee Automata it gonna creates won't change by any form in the future.
     *
     * Factory mode aim to emphasise the creation process and illuminate life cycle
     * which is my best effort to improve easy of maintenance.
     *
     * Factory itself should being destroyed after build is complete otherwise memory waste.
     *
     * @tparam EnumStatesCollection Enumeration of all states that expected to perform in the automata going to be built.
     * @tparam TupleData Tuple form data package.
     */
    template <class EnumStatesCollection, class TupleData>
    class StateAutomataBuilder {
    public:

        typedef EnumStatesCollection States;
        using FSM = vector<vector<Transition<States, TupleData>>>;
        using Item = AutomataInputManagement<TupleData>;

        /**
         * Add new reflect relationship among the automata.
         *
         * For states with multiple transitions, those added first will execute first if there's ambiguous conditions.
         *
         * @param from the state that
         * @param trans Transition
         * @return Factory itself in order to perform "chain call" grammar.
         */
        StateAutomataBuilder& transition(States from, Transition<States, TupleData> trans);

        template <template<class> class Item>
        StateAutomataBuilder& input(const char* name);

        /**
         * Build and output current automata
         *
         * CAUTIOUS: After build, factory will NOT release memory. Since graph (implemented by
         * nested vector) cost great amount of memory, unless multiple versions of automata are
         * needed (which is not very likely), object should be destroyed after calling this function
         * and get result automata.
         *
         * @param init_state The state that the result automata will start with.
         * @return The product automata
         */
        StateAutomata<EnumStatesCollection, TupleData> build(States init_state);

    private:
        FSM state_machine_;
        Item input_items_;
    };
    /*StateAutomataBuilder*/

}  // namespace control

#endif  // UICRM_STATEAUTOMATAS_H
