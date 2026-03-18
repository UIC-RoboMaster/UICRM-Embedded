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

#include "../../drivers/include/Automata/AutomataInputManagement.h"

using std::vector;

using communication::AutomataInputManagement;

namespace control {

    /*Transition*/
    using Predicate = std::function<bool(const AutomataInputManagement&)>;
    template <class EnumStatesCollection>
    struct Transition {
        Transition(Predicate c, EnumStatesCollection n) : condition(std::move(c)), next(n) {
        }

        Predicate condition;
        EnumStatesCollection next;
    };
    /*Transition*/

    /*StateAutomataBuilder*/
    /**
     * Factory class that guarantee Automata it gonna creates won't change by any form in the
     * future.
     *
     * Factory mode aim to emphasise the creation process and illuminate life cycle
     * which is my best effort to improve easy of maintenance.
     *
     * Factory itself should being destroyed after build is complete otherwise memory waste.
     *
     * @tparam EnumStatesCollection Enumeration of all states that expected to perform in the
     * automata going to be built.
     */
    template <class EnumStatesCollection>
    class StateAutomataBuilder {
      public:
        class StateAutomata;

        typedef EnumStatesCollection States;
        using FSM = vector<vector<Transition<States>>>;
        using Item = AutomataInputManagement;

        /**
         * Add new reflect relationship among the automata.
         *
         * For states with multiple transitions, those added first will execute first if there's
         * ambiguous conditions.
         *
         * @param from the state that
         * @param trans Transition
         * @return Factory itself in order to perform "chain call" grammar.
         */
        StateAutomataBuilder& transition(States from, Transition<States> trans) {
            size_t idx = static_cast<size_t>(from);
            if (idx >= state_machine_.size())
                state_machine_.resize(idx + 1);
            state_machine_[static_cast<size_t>(from)].emplace_back(trans);
            return *this;
        }
        StateAutomataBuilder& transition(States from, States to, Predicate condition) {
            return transition(from, Transition<EnumStatesCollection>(condition, to));
        }

        /**
         * Creat a component that represent an item input in automata
         *
         * component's behaviour decide by template parameter [Component]
         *
         * Parameter [item] does NOT require an instance, it requires structure segment and derive
         * type from it.
         *
         * @tparam Component The component that will determine behaviours based on items
         * @tparam Item A forward declaration of item type, derive by compiler
         * @tparam Struct A forward declaration of struct that contain item type, derive by compiler
         * @param item TYPE of the item, requires a declaration that represent structure segment
         * @param name The name u'd like to give to this item/component
         * @return Factory itself in order to perform "chain call" grammar.
         */
        template <template <class> class Component, typename Item, typename Struct>
        StateAutomataBuilder& input(Item Struct::*item, const char* name) {
            using Type = std::remove_reference_t<decltype(std::declval<Struct>().*item)>;
            input_items_.buildItem<Component, Type>(name);
            return *this;
        }
        template <template <class> class Component, typename T>
        StateAutomataBuilder& input(T&&, const char* name) {
            using Type = std::remove_cv_t<std::remove_reference_t<T>>;
            input_items_.buildItem<Component, Type>(name);
            return *this;
        }

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
        StateAutomata* build(States init_state) {
            return new StateAutomata(std::move(state_machine_), init_state,
                                     std::move(input_items_));
        }

      private:
        FSM state_machine_;
        Item input_items_;
    };
    /*StateAutomataBuilder*/

    /*StateAutomata*/
    /**
     * An automata that work based on graph.
     *
     * Transitions are depend on a condition that in a form of std::function(function pointer) which
     * its return value always boolean.
     *
     * This class should be constructed by class [control::StateAutomataBuilder].
     * It's NOT recommended that user construct Finite State Machine(FSM) without the aid of factory
     * class.
     *
     * @tparam EnumStatesCollection Enumeration of all states that expected to perform in the
     * automata.
     */
    template <class EnumStatesCollection>
    class StateAutomataBuilder<EnumStatesCollection>::StateAutomata {
      public:
        friend class StateAutomataBuilder;

        typedef EnumStatesCollection States;
        using FSM = vector<vector<Transition<States>>>;
        using Item = AutomataInputManagement;

        /**
         * Update items and drive automata to transit once.
         *
         * CAUTIONS: DO INPUT IN EXACT SAME ORDER AND TYPE THE FACTORY PREVIOUSLY BUILT.
         *
         * @tparam Ts A forward declaration of tuple, derive by compiler.
         * @param data A tuple form update data package.
         */
        template <typename... Ts>
        void input(const std::tuple<Ts...>& data) {
            input_items_.updateItems(data);
            evaluateTransitions();
        }

        /**
         * Automata(FSM) current output.
         *
         * @return automata current state.
         */
        States state() {
            return current_state_;
        }

        // TODO implement demonstrate()
        std::string demonstrate() const;

      private:
        StateAutomata(FSM machine, States init_state, Item inputs)
            : state_machine_(std::move(machine)),
              current_state_(init_state),
              input_items_(std::move(inputs)) {
        }

        const FSM state_machine_;
        States current_state_;

        Item input_items_;

        void evaluateTransitions() {
            for (auto& it : state_machine_[static_cast<size_t>(current_state_)])
                if (it.condition(input_items_)) {
                    current_state_ = it.next;
                    return;
                }
        }
    };
    /*StateAutomata*/

    template <class EnumStatesCollection>
    using Automata = typename StateAutomataBuilder<EnumStatesCollection>::StateAutomata;

    template <class EnumStatesCollection>
    using AutomataBuilder = StateAutomataBuilder<EnumStatesCollection>;

#define TRANLOGIC [](const communication::Ins& ins) -> bool

#define INTYPE(expr) decltype(expr){}

}  // namespace control

#endif  // UICRM_STATEAUTOMATAS_H
