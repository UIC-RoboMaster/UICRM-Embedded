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

#include "../../include/Automata/AutomataInputRemote.h"

namespace remote {
    template <class T>
    AutomataInputRemote<T>::AutomataInputRemote() : curr_val_(T(0)), last_val_(T(0)), last_update_(0) {}

    template <class T>
    void AutomataInputRemote<T>::updateImpl(const T* input) {
        T& input_val = *input;
        if (input_val != curr_val_)
            last_update_ = 0;
        ++last_update_;
        T temp = curr_val_;
        curr_val_ = input_val;
        last_val_ = temp;
    }

    template <class T>
    const T AutomataInputRemote<T>::get() {return curr_val_;}

    template <class T>
    bool AutomataInputRemote<T>::edge() {return curr_val_ != last_val_;}

    template <class T>
    bool AutomataInputRemote<T>::upEdge() {
        if constexpr (std::is_same_v<T, bool>) return edge();
        else return curr_val_ > last_val_;
    }

    template <class T>
    bool AutomataInputRemote<T>::downEdge() {
        if constexpr (std::is_same_v<T, bool>) return edge();
        else return curr_val_ < last_val_;
    }

    template <class T>
    const T AutomataInputRemote<T>::getDiff() {
        if constexpr (std::is_same_v<T, bool>) return edge();
        else return curr_val_ - last_val_;
    }

    template <class T>
    uint16_t AutomataInputRemote<T>::lastUpdate() {return last_update_;}
}  // namespace remote