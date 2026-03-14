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

#ifndef UICRM_AUTOMATAINPUTREMOTE_H
#define UICRM_AUTOMATAINPUTREMOTE_H

#include <cstdint>

#include "AutomataInputBase.h"

namespace remote {

    /**
     * CAUTIOUS: Should only specify to int(uint16_t etc) float or bool. Common type in remote
     * system OTHER TYPES MAY THROW EXCEPTIONS
     * TODO using C++20 feature [concept] to limit specification types.
     *
     * @tparam T channel data type
     */
    template <class T>
    class AutomataInputRemote : public communication::AutomataInputBase<T> {
    public:
        AutomataInputRemote();
        ~AutomataInputRemote() override = default;

        /**
         * @return If current val not equal to last val
         */
        virtual bool edge();

        /**
         * boolean specification: equal to edge()
         *
         * @return If current val greater than last val
         */
        virtual bool upEdge();

        /**
         * boolean specification: equal to edge()
         *
         * @return If current val smaller than last val
         */
        virtual bool downEdge();

        /**
         * boolean specification will output current xor last
         * int and float specification will output current-last
         *
         * @return Difference of current value and last value
         */
        virtual const T getDiff();

        /**
         * update and call this func otherwise real-time problem
         *
         * @return current value
         */
        virtual const T get();

        /**
         * This function not involving RTC or similar devices.
         * It only records how many times update() called since last time value changed.
         *
         * @return The UPDATE TIMES that since last value changed
         */
        virtual uint16_t lastUpdate();
    protected:
        T curr_val_;
        T last_val_;
        uint16_t last_update_;

        void updateImpl(const T* input) override;
    };

}  // namespace remote

#endif  // UICRM_AUTOMATAINPUTREMOTE_H
