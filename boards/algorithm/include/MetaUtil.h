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
// Created by gttgf on 2026/3/19.
//

#ifndef UICRM_METAUTIL_H
#define UICRM_METAUTIL_H

namespace TemplateMetaUtil {
    /*ValueList*/  // A type collection container
    template <auto... Vs>
    struct ValueList {};

    /*Contains & AppendUnique*/  // Search and remove duplicate types, reduce compile cost
    template <auto V, typename List>
    struct Contains;

    template <auto V>
    struct Contains<V, ValueList<>> : std::false_type {};

    template <auto V, auto Head, auto... Tail>
    struct Contains<V, ValueList<Head, Tail...>>
        : std::conditional_t<V == Head, std::true_type, Contains<V, ValueList<Tail...>>> {};

    template <typename List, auto V>
    struct AppendUnique;

    template <auto... Vs, auto V>
    struct AppendUnique<ValueList<Vs...>, V> {
        using type = std::conditional_t<Contains<V, ValueList<Vs...>>::value, ValueList<Vs...>,
                                        ValueList<Vs..., V>>;
    };
    /*Contains & AppendUnique*/

    /*CollectStates*/
    template <typename... Ts>
    struct CollectStates;

    template <>
    struct CollectStates<> {
        using type = ValueList<>;
    };

    template <typename T, typename... Rest>
    struct CollectStates<T, Rest...> {
      private:
        using rest = typename CollectStates<Rest...>::type;
        using with_from = typename AppendUnique<rest, T::from>::type;

      public:
        using type = typename AppendUnique<with_from, T::to>::type;
    };
    /*CollectStates*/

    /*Filter*/
    template <auto State, typename... Ts>
    struct Filter;

    template <auto State>
    struct Filter<State> {
        using type = std::tuple<>;
    };

    template <auto State, typename T, typename... Rest>
    struct Filter<State, T, Rest...> {
      private:
        using rest_type = typename Filter<State, Rest...>::type;

      public:
        using type =
            std::conditional_t<T::from == State,
                               decltype(std::tuple_cat(std::tuple<T>{}, rest_type{})), rest_type>;
    };
    /*Filter*/
}  // namespace TemplateMetaUtil

#endif  // UICRM_METAUTIL_H
