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

#include "../../drivers/include/Automata/AutomataInputManagement.h"
#include "MetaUtil.h"

using namespace TemplateMetaUtil;

using std::vector;

using control::AutomataInputManagement;

namespace control {

    /*Tags*/
    struct ForwardTag {};
    struct ReverseTag {};
    /*Tags*/

    /*FiniteStateMachine*/
    /**
     * Core component of state machine system
     *
     * @tparam States Collection of states in a form of enumeration
     * @tparam Trs Transitions that store in template
     */
    template <typename States, typename... Trs>
    class FiniteStateMachine {
      public:
        using StateList = typename CollectStates<Trs...>::type;

        /*interface*/  // with [Automata]
        FiniteStateMachine(States init) : current_state_(init) {
        }

        template <typename Ins>
        void step(const Ins& ins) {
            stepImpl(ins, StateList{});
        }

        States state() const {
            return current_state_;
        }
        /*interface*/
      private:
        States current_state_;

        template <typename Ins, auto... St>
        void stepImpl(const Ins& ins, ValueList<St...>) {
            States res = current_state_;
            ((current_state_ == St ? (res = evalState<St>(ins), void()) : void()), ...);
            current_state_ = res;
        }

        template <auto St, typename Ins>
        States evalState(const Ins& ins) {
            using EvalGroup = typename Filter<St, Trs...>::type;
            return evalTuple<EvalGroup>(ins,
                                        std::make_index_sequence<std::tuple_size_v<EvalGroup>>{});
        }

        template <typename EvalGroup, typename Ins, size_t... Index>
        States evalTuple(const Ins& ins, std::index_sequence<Index...>) {
            bool made_transit = false;
            States res = current_state_;
            ((!made_transit && std::tuple_element_t<Index, EvalGroup>::eval(ins)
                  ? (res = std::tuple_element_t<Index, EvalGroup>::to, made_transit = true)
                  : made_transit),
             ...);
            return res;
        }
    };
    /*FiniteStateMachine*/

    /*Transition*/
    template <auto From, auto To, typename Fn, typename ReTag>
    struct Transition {
        static constexpr auto from = From;
        static constexpr auto to = To;
        template <typename Ins>
        static bool eval(const Ins& ins) {
            return std::is_same_v<ReTag, ReverseTag> ? !Fn{}(ins) : Fn{}(ins);
        }
    };
    /*Transition*/

    /*CollectTransitions*/
    template <typename EnumStatesCollection, typename... Trs>
    struct CollectTransitions {
        template <auto From, auto To, typename ReTag, typename Fn>
        constexpr auto addTrans(Fn) const {
            using NewTr = Transition<From, To, Fn, ReTag>;
            return CollectTransitions<EnumStatesCollection, Trs..., NewTr>{};
        }

        // DEBUG ONLY
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
     * Transitions are depend on a condition that in a form of custom (static) functions which
     * their return value always boolean.
     *
     * This class should be constructed by factory tool [AutomataBuilder].
     * It's NOT recommended that user construct Finite State Machine(FSM) without the aid of factory
     * class.
     *
     * @tparam FSM Finite State Machine
     * @tparam DMS Data Management System
     */
    template <typename FSM, typename DMS>
    class Automata {
      public:
        template <typename StateType>
        explicit constexpr Automata(StateType init_state) : state_machine_(init_state), comps_() {
        }

        /**
         * Update items and drive automata to transit once.
         *
         * Data shall update in tuple package should fit exactly same order to
         * the order they registered.
         *
         * CAUTIOUS: There's input size and type check, but wrong input fit above features
         * can't detect by compiler.
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
        DMS comps_;
    };
    /*Automata*/

    /*AutomataBuilder*/
    /**
     * Factory tool that aid to construct class [Automata].
     *
     * @tparam EnumStatesCollection Collection of states in a form of enumeration
     * @tparam Items Registered Inputs store here
     * @tparam Trans Defined transitions store here
     */
    template <typename EnumStatesCollection, typename Items = CollectItems<>,
              typename Trans = CollectTransitions<EnumStatesCollection>>
    struct AutomataBuilder {
        Items items_;
        Trans trans_;

        /**
         * Register input type and interface that store and interact with data.
         *
         * Can be access later by index.
         * The order of registration is the index of item (start from 0).
         *
         * Component must be a template class with function [update()] and tParam [T].
         * It's recommend to use classes derived from base class [AutomataInputComponentsBase].
         * There's no extra cost on virtual-related features so no worry.
         *
         * @tparam Component User select component that decides how data store and interact.
         * @tparam T A forward declaration of register variable/enumeration type
         * @param v Forward declaration
         * @return The new builder that has updated type.
         */
        template <template <class> class Component, typename T>
        constexpr auto item(T&& v) const {
            auto new_items = items_.template addItem<Component>(std::forward<T>(v));
            using NewItems = decltype(new_items);
            return AutomataBuilder<EnumStatesCollection, NewItems, Trans>{new_items, trans_};
        }

        /**
         * Register input type and interface that store and interact with data.
         *
         * Can be access later by index.
         * The order of registration is the index of item (start from 0).
         *
         * @tparam Component User select component that decides how data store and interact.
         * @tparam Struct A forward declaration of register struct type
         * @tparam Member A forward declaration of register struct member type
         * @param member Forward declaration
         * @return The new builder that has updated type.
         */
        template <template <class> class Component, typename Struct, typename Member>
        constexpr auto item(Member Struct::*member) const {
            auto new_items = items_.template addItem<Component>(member);
            using NewItems = decltype(new_items);
            return AutomataBuilder<EnumStatesCollection, NewItems, Trans>{new_items, trans_};
        }

        /**
         * Define a conditional transition(edge) in the Automata(FSM).
         *
         * @tparam From Which state this transition begins with.
         * @tparam To Which state this transition goes to.
         * @tparam Logic Transition encapsulation type.
         * @param logic Transition logic
         * @param ReTag[anonymous] if reverse logic output
         * @return The new builder that has updated type.
         */
        template <auto From, auto To, typename Logic, typename ReTag = ForwardTag>
        constexpr auto transition(Logic logic, ReTag = {}) const {
            using NewTrans = decltype(trans_.template addTrans<From, To, ReTag>(logic));
            return AutomataBuilder<EnumStatesCollection, Items, NewTrans>{items_, NewTrans{}};
        }

        /**
         * Output produced automata
         *
         * Also stop "chain call" grammar due to return type changing
         *
         * @tparam init_state State that Automata will begin with.
         * @return A built Automata
         */
        template <auto init_state>
        constexpr auto build() const {
            using StateSystem = typename Trans::rebind_to_fsm;
            using InputSystem = typename Items::rebind_to_management;
            return Automata<StateSystem, InputSystem>(init_state);
        }

        /**
         * Output produced automata in a form of pointer point to dynamic allocated [Automata] obj.
         *
         * Also stop "chain call" grammar due to return type changing.
         *
         * It is recommended to use [build()] instead of this.
         * Heap allocation will lead to extra time-space cost.
         *
         * @tparam init_state State that Automata will begin with.
         * @return A built Automata
         */
        template <auto init_state>
        constexpr auto* build_heap_allocation() const {
            using StateSystem = typename Trans::rebind_to_fsm;
            using InputSystem = typename Items::rebind_to_management;
            return new Automata<StateSystem, InputSystem>(init_state);
        }
    };
    /*AutomataBuilder*/

#define TRANLOGIC [](const auto& ins) -> bool

#define COMPONENT(index) (ins.template get<index>())

}  // namespace control

#endif  // UICRM_AUTOMATASYSTEM_H
