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

#include <utility>

namespace riot {
namespace navigation {

namespace detail {
template <class F, class... Args, class stream_traits, class charT,
          class traits = std::char_traits<charT>>
auto &manip_impl(basic_gnss_istream<stream_traits, charT, traits> &strm, F f,
                 Args &&... args) {
  strm.set(f(std::forward<Args>(args)...));
  return strm;
}
}

#define MANIP(name)                                                            \
  template <class stream_traits, class charT,                                  \
            class traits = std::char_traits<charT>>                            \
  auto &name(basic_gnss_istream<stream_traits, charT, traits> &strm) {         \
    return detail::manip_impl(strm, stream_traits::name);                      \
  }

MANIP(skipchecksum)
MANIP(_1hz)
MANIP(_2hz)
MANIP(_4hz)
MANIP(_5hz)
MANIP(_8hz)
MANIP(_10hz)
MANIP(baud_rate_4800)
MANIP(baud_rate_9600)
MANIP(baud_rate_19200)
MANIP(baud_rate_38400)
MANIP(baud_rate_57600)
MANIP(baud_rate_115200)
MANIP(skytraq_binary)
MANIP(skytraq_nmea)
MANIP(car)
MANIP(crc)
MANIP(disable_waas)
MANIP(enable_waas)
MANIP(factory_defaults)
MANIP(nmea_default)
MANIP(off)
MANIP(pedestrian)
MANIP(power_normal)
MANIP(power_saver)
MANIP(query_ephemeris)
MANIP(query_navigation_mode)
MANIP(update_rate)
MANIP(query_waas)
MANIP(release)
MANIP(rmc_only)
}
}
