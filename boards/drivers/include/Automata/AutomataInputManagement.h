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

#include <vector>
#include <memory>

#include "AutomataInputBase.h"
#include "StateAutomatas.h"

namespace communication {

    template<class TupleData>
    class AutomataInputManagement {
    public:
        /**
         * Add a new component.
         *
         * Should (only) call by [StateAutomataBuilder].
         *
         * @tparam Index
         * @tparam Component
         * @param name
         */
        template <size_t Index, template <class> class Component>
        void buildItem(const char* name = "");

        /**
         * update all components.
         * @param data A tuple contians all items that components need to update
         */
        void updateItems(const TupleData& data);

        /**
         * A type-safe interface for obtaining polymorphism-template class(components).
         *
         * Directly return [AutomataInput] will cause user use dynamic_cast<>()
         * RTTI, which not often available in embedded system support dynamic_cast<>().
         * Using template parameter in order to not depend on RTTI.
         *
         * @tparam Index Where the component that represent the item locate.
         * @return Component that represent the named item.
         */
        template <size_t Index>
        auto& get();
        auto& get(size_t index);

        /**
         * @param name Items' custom name.
         * @return Index where the component that represent the named item.
         */
        size_t getIndexByName(std::string& name);

        /**
         * @param name Items' custom name.
         * @return Component that represent the named item.
         */
        AutomataInput& getByName(std::string& name);
    private:
        std::vector<std::unique_ptr<AutomataInput>> items_;

        template <size_t... Index>
        void updateItemsImpl(const TupleData& data, std::index_sequence<Index...>);
    };

    template<class TupleData>
    using Ins = AutomataInputManagement<TupleData>;

}  // namespace communication

#endif  // UICRM_AUTOMATAINPUTMANAGEMENT_H
