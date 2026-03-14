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
    template <class TupleData>
    template <size_t Index, template <class> class Component>
    void AutomataInputManagement<TupleData>::buildItem(const char* name) {
        using T = std::tuple_element_t<Index, TupleData>;
        items_.emplace_back(std::make_unique<Component<T>()>);
        static_cast<Component<T>&>(*items_.back())->setName(name);
    }

    template <class TupleData>
    void AutomataInputManagement<TupleData>::updateItems(const TupleData& data) {
        updateImpl(data, std::make_index_sequence<std::tuple_size_v<TupleData>>{});
    }

    template <class TupleData>
    template <size_t... Index>
    void AutomataInputManagement<TupleData>::updateItemsImpl(const TupleData& data, std::index_sequence<Index...>) {
        ((items_[Index])->update(&std::get<Index>(data)), ...);
    }

    template <class TupleData>
    template <size_t Index>
    auto& AutomataInputManagement<TupleData>::get() {return *items_[Index];}

    //TODO name related implementation
    //
    // template <class TupleData>
    // AutomataInput& AutomataInputManagement<TupleData>::getByName(std::string& name) {
    //
    // }
    //
    // template <class TupleData>
    // size_t AutomataInputManagement<TupleData>::getIndexByName(std::string& name) {
    //
    // }
}  // namespace communication