#pragma once

//
// Copyright (C) 2015-2016 Cryogenic Graphics Ltd
//

/*
    This file is part of Attempto.

    Attempto is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    Attempto is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with Attempto.  If not, see <http://www.gnu.org/licenses/>.
*/

namespace riot {
namespace navigation {
namespace detail {

inline constexpr auto char_as_digit(char c) { return c - '0'; }

template <class... Ts> inline constexpr auto char_sequence_as_int(Ts... ts);

template <> inline constexpr auto char_sequence_as_int() { return 0; }

template <class T, class... Ts>
inline constexpr auto char_sequence_as_int(T t, Ts... ts) {
  return char_as_digit(t) + char_sequence_as_int(ts...) * 10;
}

inline constexpr auto shifter() noexcept { return 0; }

template <class T, class... Ts, size_t n = 8>
inline constexpr auto shifter(T t, Ts... ts) noexcept {
  return t << sizeof...(ts)*n | shifter(ts...);
}
}
}
}
