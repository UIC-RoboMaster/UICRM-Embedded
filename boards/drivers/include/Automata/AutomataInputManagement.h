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

#ifndef UICRM_AUTOMATAINPUTMANAGEMENT_H
#define UICRM_AUTOMATAINPUTMANAGEMENT_H

#include <memory>
#include <vector>

#include "AutomataInputComponents.h"

namespace control {
    template <typename... Components>
    class AutomataInputManagement {
      public:
        // AutomataInputManagement() = default;
        /**
         * Add a new component.
         *
         * Should (only) call by [AutomataBuilder].
         *
         * @tparam Component
         * @tparam Type
         * @param name
         */
        // template <template <class> class Component, typename Type>
        // void buildItem(const char* name) {
        //     items_.emplace_back(std::make_unique<Component<Type>>(name));
        // }

        /**
         * update all components.
         * @param data A tuple contians all items that components need to update
         */
        template <typename... Ts>
        constexpr void updateItems(const std::tuple<Ts...>& data) {
            static_assert(sizeof...(Ts) == std::tuple_size_v<decltype(items_)>, "Automata: input size mismatch");
            updateItemsImpl(data, std::index_sequence_for<Ts...>{});
        }

        /**
         * Get component
         *
         * CAUTIOUS: PLEASE DO MAKE SURE THAT [ReturnType] IS MATCHING WITH WHAT COMPONENT TRULY IS
         * UNDEFINED BEHAVIOUR WILL OCCUR IF TYPE UNMATCH.
         *
         * Due to forbidden of RTTI in embedded system, dynamic_cast<>() is not available.
         * Polymorphism type here is NOT safe.
         *
         * Implement in-class due to compiler type check
         *
         * @tparam Struct
         * @tparam Member
         * @tparam Component
         * @param index
         * @param member
         * @return
         */
        // template <template <class> class Component, typename Struct, typename Member>
        // auto get(Member Struct::*member, size_t index) const
        //     -> Component<std::remove_reference_t<decltype(((Struct*)nullptr)->*member)>>& {
        //     using Type = std::remove_reference_t<decltype(((Struct*)nullptr)->*member)>;
        //     return static_cast<Component<Type>&>(*items_[index]);
        // }
        template <size_t I>
        constexpr const auto& get() const {
            return std::get<I>(items_);
        }

        /**
         * @param name Items' custom name.
         * @return Index where the component that represent the named item.
         */
        // TODO name related implementation
        //
        // template <class ReturnType>
        // AutomataInput& AutomataInputManagement::getByName(std::string& name) {
        //
        // }

        /**
         * @param name Items' custom name.
         * @return Component that represent the named item.
         */
        // size_t AutomataInputManagement::getIndexByName(std::string& name) {
        //
        // }

      private:
        std::tuple<Components...> items_;

        template <typename... Ts, size_t... Index>
        constexpr void updateItemsImpl(const std::tuple<Ts...>& data, std::index_sequence<Index...>) {
            (std::get<Index>(items_).update(std::get<Index>(data)), ...);
        }
    };

    /*CollectItems*/
    template<typename... Items>
    struct CollectItems {
        template<template<class> class Component, typename T>
        constexpr auto addItem(T&& v) const {
            using Type = std::remove_cv_t<std::remove_reference_t<decltype(v)>>;
            using NewComponent = Component<Type>;
            return CollectItems<Items..., NewComponent>{};
        }
        template<template<class> class Component, typename Struct, typename Member>
        constexpr auto addItem(Member Struct::*member) const {
            using Type = std::remove_cv_t<std::remove_reference_t<Member>>;
            using NewComponent = Component<Type>;
            return CollectItems<Items..., NewComponent>{};
        }

        //DEBUG ONLY
        auto output() {
            return AutomataInputManagement<Items...>{};
        }

        using rebind_to_management = AutomataInputManagement<Items...>;
    };
    /*CollectItems*/

    template<typename... Items>
    using Ins = AutomataInputManagement<Items...>;

}  // namespace communication

#endif  // UICRM_AUTOMATAINPUTMANAGEMENT_H
