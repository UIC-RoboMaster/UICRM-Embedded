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
        /**
         * Update all components.
         *
         * @param data A tuple contians all items that components need to update
         */
        template <typename... Ts>
        constexpr void updateItems(const std::tuple<Ts...>& data) {
            static_assert(sizeof...(Ts) == std::tuple_size_v<decltype(items_)>, "Automata: input size mismatch");
            updateItemsImpl(data, std::index_sequence_for<Ts...>{});
        }

        /**
         * Get component.
         *
         * @tparam Index That fixed in building process.
         * @return Component.
         */
        template <size_t Index>
        constexpr const auto& get() const {
            return std::get<Index>(items_);
        }

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
