#pragma once

//
// Copyright (C) 2015-2016 Cryogenic Graphics Ltd
//

/*
    This file is part of Reliving.

    Reliving is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    Reliving is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with Reliving.  If not, see <http://www.gnu.org/licenses/>.
*/

#include <array>
#include <cstring>
#include <stdexcept>

namespace riot {
namespace exception {

template <int N = 256> auto strerrno(int err) {
  std::array<char, N> error_buf{{'\0'}};
  return std::string(strerror_r(err, error_buf.data(), error_buf.size()));
}
}
}
