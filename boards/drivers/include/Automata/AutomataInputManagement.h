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
        AutomataInputManagement();
        ~AutomataInputManagement();

        void buildItems();

        void updateItems(const TupleData& data);

        AutomataInputBase& get(size_t index);
    private:
        std::vector<std::unique_ptr<AutomataInputBase>> inputs_;
    };

}  // namespace communication

#endif  // UICRM_AUTOMATAINPUTMANAGEMENT_H
