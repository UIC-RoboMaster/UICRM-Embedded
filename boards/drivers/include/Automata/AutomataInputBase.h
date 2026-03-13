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
     * A general base class for automata input
     *
     * Its derive classes should represent a single channel/value/variable affecting automata
     * transition.
     *
     * Class [AutomataInputManagement] should include this class family to perform interaction with
     * automata.
     */
    class AutomataInputBase {
    public:
        virtual ~AutomataInputBase() = default;
        virtual void update() = 0;
        virtual string name() {return name_;};
    protected:
        string name_;
    };

}  // namespace communication

#endif  // UICRM_AUTOMATAINPUTBASE_H
