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

#include "../../include/Automata/AutomataInputManagement.h"

namespace communication {
    template <template <class> class Component, typename Type>
    void AutomataInputManagement::buildItem(const std::string& name) {
        items_.emplace_back(std::make_unique<Component<Type>>(name));
    }

    template <typename... Ts>
    void AutomataInputManagement::updateItems(const std::tuple<Ts...>& data) {
        updateItemsImpl(data, std::make_index_sequence<sizeof...(Ts)>{});
    }

    template <typename... Ts, size_t... Index>
    void AutomataInputManagement::updateItemsImpl(const std::tuple<Ts...>& data, std::index_sequence<Index...>) {
        (items_[Index]->update(&std::get<Index>(data)), ...);
    }

    // template <size_t Index>
    // auto& AutomataInputManagement::get() {return *items_[Index];}

    template <class ReturnType>
    auto& AutomataInputManagement::get(const size_t index) {return static_cast<ReturnType>(*items_[index]);}


    //TODO name related implementation
    //
    // template <class ReturnType>
    // AutomataInput& AutomataInputManagement::getByName(std::string& name) {
    //
    // }
    //
    // size_t AutomataInputManagement::getIndexByName(std::string& name) {
    //
    // }
}  // namespace communication