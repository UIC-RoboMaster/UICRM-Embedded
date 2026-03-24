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

#ifndef UICRM_AUTOMATAINPUTCOMPONENTBASE_H
#define UICRM_AUTOMATAINPUTCOMPONENTBASE_H

#include <string>
#include <cstdint>

using std::string;

namespace control {
    /**
     * A general pure virtual base class for automata input
     * Components are recommend to be derived by this class
     *
     * Its derive classes should represent a single channel/value/variable affecting automata
     * transition.
     *
     * Class [AutomataInputManagement] include this class family to perform interaction with
     * automata.
     */
    template <typename T>
    class AutomataInputComponentsBase {
      public:
        AutomataInputComponentsBase() = default;
        virtual ~AutomataInputComponentsBase() = default;
        virtual void update(const T& input) = 0;
    };

    // TODO All components using C++20 feature [concept] to limit specification types.

    /**
     * Simply store raw value
     *
     * @tparam T type
     */
    template <class T>
    class AutomataInputRaw : public AutomataInputComponentsBase<T> {
    public:
        ~AutomataInputRaw() override = default;
        virtual void update(const T& input) {
            val_ = input;
        }
        virtual T get() const {return val_;}
    protected:
        T val_;
    };

    /**
     * Component that can specify edge and detect latency based on updated times.
     *
     * @tparam T type
     */
    template <class T>
    class AutomataInputRemote : public AutomataInputComponentsBase<T> {
      public:
        ~AutomataInputRemote() override = default;

        /**
         * record previous value and last update time.
         *
         * @param input
         */
        void update(const T& input) {
            if (input != curr_val_)
                last_update_ = 0;
            ++last_update_;
            T temp = curr_val_;
            curr_val_ = input;
            last_val_ = temp;
        }

        /**
         * @return If current val not equal to last val
         */
        bool edge() const {
            return curr_val_ != last_val_;
        }

        /**
         * boolean specification: equal to edge()
         *
         * @return If current val greater than last val
         */
        virtual bool upEdge() const {
            if constexpr (std::is_same_v<T, bool>)
                return edge();
            else
                return curr_val_ > last_val_;
        }

        /**
         * boolean specification: equal to edge()
         *
         * @return If current val smaller than last val
         */
        virtual bool downEdge() const {
            if constexpr (std::is_same_v<T, bool>)
                return edge();
            else
                return curr_val_ < last_val_;
        }

        /**
         * boolean specification will output current xor last
         * int and float specification will output current-last
         *
         * @return Difference of current value and last value
         */
        virtual const T getDiff() const {
            if constexpr (std::is_same_v<T, bool>)
                return edge();
            else
                return curr_val_ - last_val_;
        }

        /**
         * update and call this func otherwise real-time problem
         *
         * @return current value
         */
        virtual T get() const {
            return curr_val_;
        }

        /**
         * This function not involving RTC or similar devices.
         * It only records how many times update() called since last time value changed.
         *
         * @return The UPDATE TIMES that since last value changed
         */
        virtual uint16_t lastUpdate() const {
            return last_update_;
        }

      protected:
        T curr_val_;
        T last_val_;
        uint16_t last_update_;
    };

}  // namespace communication

#endif  // UICRM_AUTOMATAINPUTCOMPONENTBASE_H
