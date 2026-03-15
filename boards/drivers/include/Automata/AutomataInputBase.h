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

#ifndef UICRM_AUTOMATAINPUTBASE_H
#define UICRM_AUTOMATAINPUTBASE_H

#include <string>

using std::string;

namespace communication {

    /**
     * A polymorphism interface for automata input item
     */
    class AutomataInput {
    public:
        virtual ~AutomataInput() = 0;
        virtual void update(const void*) = 0;
    };

    /**
     * A general pure virtual base class for automata input
     * Every automata input component should derive by this class
     *
     * Its derive classes should represent a single channel/value/variable affecting automata
     * transition.
     *
     * Class [AutomataInputManagement] include this class family to perform interaction with automata.
     */
    template <typename T>
    class AutomataInputBase : public AutomataInput {
    public:
        explicit AutomataInputBase(const char* name) : name_(name) {}
        virtual ~AutomataInputBase() override = default;
        void update (const void* input) final {updateImpl(static_cast<const T*>(input));}
        virtual string name() final {return name_;}
    protected:
        string name_;

        virtual void updateImpl(const T* input) = 0;
    };

}  // namespace communication

#endif  // UICRM_AUTOMATAINPUTBASE_H
