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

namespace communication {

    class AutomataInputManagement {
    public:
        /**
         * Add a new component.
         *
         * Should (only) call by [StateAutomataBuilder].
         *
         * @tparam Component
         * @tparam Type
         * @param name
         */
        template <template<class> class Component, typename Type>
        void buildItem(const std::string& name);

        /**
         * update all components.
         * @param data A tuple contians all items that components need to update
         */
        template <typename... Ts>
        void updateItems(const std::tuple<Ts...>& data);

        /**
         * Get component
         *
         * CAUTIOUS: PLEASE DO MAKE SURE THAT [ReturnType] IS MATCHING WITH WHAT COMPONENT TRULY IS
         * UNDEFINED BEHAVIOUR WILL OCCUR IF TYPE UNMATCH.
         *
         * Due to forbidden of RTTI in embedded system, dynamic_cast<>() is not available.
         * Polymorphism type here is NOT safe.
         *
         * @tparam ReturnType Actual component type
         * @param index where
         * @return A [ReturnType] type component
         */
        template <class ReturnType>
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
        template <class ReturnType>
        AutomataInput& getByName(std::string& name);
    private:
        std::vector<std::unique_ptr<AutomataInput>> items_;

        template <typename... Ts, size_t... Index>
        void updateItemsImpl(const std::tuple<Ts...>& data, std::index_sequence<Index...>);
    };

    using Ins = AutomataInputManagement;

}  // namespace communication

#endif  // UICRM_AUTOMATAINPUTMANAGEMENT_H
