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

#ifndef UICRM_AUTOMATASYSTEM_H
#define UICRM_AUTOMATASYSTEM_H

#include <functional>
#include <string>
#include <vector>

#include "MetaUtil.h"
#include "../../drivers/include/Automata/AutomataInputManagement.h"

using namespace TemplateMetaUtil;

using std::vector;

using control::AutomataInputManagement;

namespace control {

    /*FiniteStateMachine*/
    template <typename States, typename... Trs>
    class FiniteStateMachine {
    public:
        using StateList = typename CollectStates<Trs...>::type;

        /*interface*/           // with [Automata]
        FiniteStateMachine(States init) : current_state_(init) {}

        template<typename Ins>
        void step(const Ins& ins) {
            stepImpl(ins, StateList{});
        }

        States state() const {return current_state_;}
        /*interface*/
    private:
        States current_state_;

        template<typename Ins, auto... St>
        void stepImpl(const Ins& ins, ValueList<St...>) {
            States res = current_state_;
            ((current_state_ == St ? (res = evalState<St>(ins), void()) : void()), ...);
            current_state_ = res;
        }

        template<auto St, typename Ins>
        States evalState(const Ins& ins) {
            using EvalGroup = typename Filter<St, Trs...>::type;
            return evalTuple<EvalGroup>(ins, std::make_index_sequence<std::tuple_size_v<EvalGroup>>{});
        }

        template<typename EvalGroup, typename Ins, size_t... Index>
        States evalTuple(const Ins& ins, std::index_sequence<Index...>) {
            bool made_transit = false;
            States res = current_state_;
            ((!made_transit && std::tuple_element_t<Index, EvalGroup>::eval(ins) ?
                (res = std::tuple_element_t<Index, EvalGroup>::to, made_transit = true) : made_transit),
                ...);
            return res;
        }
    };
    /*FiniteStateMachine*/

    /*Transition*/
    template<auto From, auto To, typename Fn>
    struct Transition {
        static constexpr auto from = From;
        static constexpr auto to   = To;
        // Fn fn;   //C++17 can't construct lambda by default, need to store an instance.
        template<typename Ins>
        static bool eval(const Ins& ins) {
            return Fn{}(ins);    //Performance optimization if C++20(or +) in future
            // return fn(ins);
        }
    };
    /*Transition*/

    /*CollectTransitions*/
    template<typename EnumStatesCollection, typename... Trs>
    struct CollectTransitions {
        template<auto From, auto To, typename Fn>
        constexpr auto addTrans(Fn) const {
            using NewTr = Transition<From, To, Fn>;
            return CollectTransitions<EnumStatesCollection, Trs..., NewTr>{};
        }

        //DEBUG ONLY
        auto output(EnumStatesCollection init_state) {
            return FiniteStateMachine<EnumStatesCollection, Trs...>(init_state);
        }

        using rebind_to_fsm = FiniteStateMachine<EnumStatesCollection, Trs...>;
    };
    /*CollectTransitions*/

    /*Automata*/
    /**
     * An automata that work based on graph.
     *
     * Transitions are depend on a condition that in a form of function pointer which
     * its return value always boolean.
     *
     * This class should be constructed by factory function [make_automata].
     * It's NOT recommended that user construct Finite State Machine(FSM) without the aid of factory
     * class.
     *
     * @tparam FSM
     * @tparam Inputs
     */
    template <typename FSM, typename Components>
    class Automata {
      public:
        template<typename StateType>
        explicit constexpr Automata(StateType init_state)
            : state_machine_(init_state), comps_() {}

        /**
         * Update items and drive automata to transit once.
         *
         * CAUTIONS: DO INPUT IN EXACT SAME ORDER AND TYPE THE FACTORY PREVIOUSLY BUILT.
         *
         * @tparam Ts A forward declaration of tuple, derive by compiler.
         * @param data A tuple form update data package.
         */
        template <typename... Ts>
        constexpr void input(const std::tuple<Ts...>& data) {
            comps_.updateItems(data);
            state_machine_.step(comps_);
        }

        /**
         * Automata(FSM) current output.
         *
         * @return automata current state.
         */
        constexpr auto state() const {
            return state_machine_.state();
        }

        // TODO implement demonstrate()
        std::string demonstrate() const;

      private:
        FSM state_machine_;
        Components comps_;
    };
    /*Automata*/

    /*AutomataBuilder*/
    template<
    typename EnumStatesCollection,
    typename Items = CollectItems<>,
    typename Trans = CollectTransitions<EnumStatesCollection>>
    struct AutomataBuilder {
        Items items_;
        Trans trans_;

        template <template<class> class Component, typename T>
        constexpr auto item(T&& v) const {
            auto new_items = items_.template addItem<Component>(std::forward<T>(v));
            using NewItems = decltype(new_items);
            return AutomataBuilder<EnumStatesCollection, NewItems, Trans>{new_items, trans_};
        }

        template <template<class> class Component, typename Struct, typename Member>
        constexpr auto item(Member Struct::*member) const {
            auto new_items = items_.template addItem<Component>(member);
            using NewItems = decltype(new_items);
            return AutomataBuilder<EnumStatesCollection, NewItems, Trans>{new_items, trans_};
        }

        template <auto From, auto To, typename Lambda>
        constexpr auto transition(Lambda lambda) const {
            using NewTrans = decltype(trans_.template addTrans<From, To>(lambda));
            return AutomataBuilder<EnumStatesCollection, Items, NewTrans>{items_, NewTrans{}};
        }

        template <auto init_state>
        constexpr auto build() const {
            using StateSystem = typename Trans::rebind_to_fsm;
            using InputSystem = typename Items::rebind_to_management;
            return Automata<StateSystem, InputSystem>(init_state);
        }
    };
    /*AutomataBuilder*/

#define TRANLOGIC [](const auto& ins) -> bool

}  // namespace control

#endif  // UICRM_AUTOMATASYSTEM_H
