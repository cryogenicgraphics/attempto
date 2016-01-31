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

#include <array>
#include <chrono>
#include <cmath>
#include <iomanip>
#include <sstream>
#include <type_traits>
#include <utility>
#include <vector>

#include "basic_utility.hpp"

namespace riot {
namespace navigation {

using system_clock = std::chrono::system_clock;
using time_point = std::chrono::time_point<system_clock>;

enum class cardinal_point : char { north, south, east, west };

enum class gnss_quality_indicator : char {
  invalid = 0,
  fix,
  differential,
  sensitive
};

enum class fix_geometry : char { invalid = 1, _2D = 2, _3D = 3 };

enum class fix_type : char {
  autonomous,
  differential,
  estimated,
  not_valid,
  simulator
};

enum class fix_mode : char { autonomous, manual };

enum class fix_mode_skytraq : char {
  invalid = 0,
  _2D = 1,
  _3D = 2,
  _3D_DGNSS = 3
};

enum class status : char { inactive, active };

namespace detail {
template <class FwdIt, class T>
constexpr std::enable_if_t<std::is_same<T, time_point>::value>
extract(FwdIt &it, FwdIt, T &t) noexcept {
  auto to_utc = [](auto beg_, auto it_) {
    auto fn = [&] {
      auto u = seqchar_int(beg_[1], beg_[0], '0');
      beg_ += 2;
      return u;
    };

    const auto siz = it_ - beg_;

    auto h = fn();
    auto m = fn();
    auto s = fn();
    ++beg_;

    int ms{0};
    switch (siz) {
    default:
      __builtin_unreachable();
    case sizeof("hhmmss.sss") - 1: {
      ms = seqchar_int(beg_[2], beg_[1], beg_[0]);
      break;
    }
    case sizeof("hhmmss.ss") - 1: {
      ms = seqchar_int(beg_[1], beg_[0], '0');
      break;
    }
    case sizeof("hhmmss.s") - 1: {
      ms = seqchar_int(beg_[0], '0', '0');
      break;
    }
    case sizeof("hhmmss") - 1: {
      break;
    }
    }
    return time_point(std::chrono::hours(h) + std::chrono::minutes(m) +
                      std::chrono::seconds(s) + std::chrono::milliseconds(ms));
  };

  FwdIt beg = it;
  while (*it != ',') {
    ++it;
  }
  t = to_utc(beg, it);
}

template <class FwdIt, class T>
constexpr std::enable_if_t<std::is_floating_point<T>::value>
extract(FwdIt &beg, FwdIt, T &t) noexcept {
  using remove_const_pointer_t =
      std::add_pointer_t<std::remove_const_t<std::remove_pointer_t<FwdIt>>>;
  remove_const_pointer_t last{nullptr};
  t = std::strtof(beg, &last);
  beg = last;
}

template <class FwdIt, class T>
constexpr std::enable_if_t<std::is_same<T, short>::value>
extract(FwdIt &it, FwdIt, T &t) noexcept {
  std::size_t pos{0};
  t = static_cast<T>(std::stoi(it, &pos));
  it += pos;
}

template <class FwdIt, class T>
constexpr std::enable_if_t<std::is_same<T, std::string>::value>
extract(FwdIt &, FwdIt, T &t) noexcept {
  t = std::string();
}

template <class FwdIt, class T>
constexpr std::enable_if_t<std::is_same<T, char>::value>
extract(FwdIt &it, FwdIt, T &t) noexcept {
  t = *it++;
}

template <class FwdIt, class T>
constexpr std::enable_if_t<std::is_same<T, cardinal_point>::value>
extract(FwdIt &it, FwdIt, T &t) noexcept {
  switch (*it) {
  default:
    __builtin_unreachable();
  case 'N':
    t = cardinal_point::north;
    break;
  case 'S':
    t = cardinal_point::south;
    break;
  case 'E':
    t = cardinal_point::east;
    break;
  case 'W':
    t = cardinal_point::west;
    break;
  }
  ++it;
}

template <class FwdIt, class T>
constexpr std::enable_if_t<std::is_same<T, fix_geometry>::value>
extract(FwdIt &it, FwdIt, T &t) noexcept {
  switch (*it) {
  default:
    __builtin_unreachable();
  case '1':
    t = fix_geometry::invalid;
    break;
  case '2':
    t = fix_geometry::_2D;
    break;
  case '3':
    t = fix_geometry::_3D;
    break;
  }
  ++it;
}

template <class FwdIt, class T>
constexpr std::enable_if_t<std::is_same<T, fix_mode>::value>
extract(FwdIt &it, FwdIt, T &t) noexcept {
  switch (*it) {
  default:
    __builtin_unreachable();
  case 'A':
    t = fix_mode::autonomous;
    break;
  case 'M':
    t = fix_mode::manual;
    break;
  }
  ++it;
}

template <class FwdIt, class T>
constexpr std::enable_if_t<std::is_same<T, fix_type>::value>
extract(FwdIt &it, FwdIt, T &t) noexcept {
  switch (*it) {
  default:
    __builtin_unreachable();
  case 'A':
    t = fix_type::autonomous;
    break;
  case 'D':
    t = fix_type::differential;
    break;
  case 'E':
    t = fix_type::estimated;
    break;
  case 'N':
    t = fix_type::not_valid;
    break;
  case 'S':
    t = fix_type::simulator;
    break;
  }
  ++it;
}

template <class FwdIt, class T>
constexpr std::enable_if_t<std::is_same<T, gnss_quality_indicator>::value>
extract(FwdIt &it, FwdIt, T &t) noexcept {
  switch (*it) {
  default:
    __builtin_unreachable();
  case '0':
    t = gnss_quality_indicator::invalid;
    break;
  case '1':
    t = gnss_quality_indicator::fix;
    break;
  case '2':
    t = gnss_quality_indicator::differential;
    break;
  case '3':
    t = gnss_quality_indicator::sensitive;
    break;
  }
  ++it;
}

template <class FwdIt, class T>
constexpr std::enable_if_t<std::is_same<T, status>::value>
extract(FwdIt &it, FwdIt, T &t) noexcept {
  switch (*it) {
  default:
    __builtin_unreachable();
  case 'V':
    t = status::inactive;
    break;
  case 'A':
    t = status::active;
    break;
  }
  ++it;
}

template <class FwdIt, class T>
constexpr std::enable_if_t<std::is_same<T, struct tm>::value>
extract(FwdIt &it, FwdIt, T &t) noexcept {
  memset(&t, 0, sizeof(struct tm));
  t.tm_mday = seqchar_int(it[1], it[0], '0');
  it += 2;
  t.tm_mon = seqchar_int(it[1], it[0], '0') - 1;
  it += 2;
  t.tm_year = seqchar_int(it[1], it[0], '0') + 2000 - 1900;
  it += 2;
}

template <class FwdIt, class... Ts>
constexpr void nmea_scanner_helper(FwdIt &, FwdIt, Ts &...) noexcept;

template <class FwdIt>
inline constexpr void nmea_scanner_helper(FwdIt &, FwdIt) noexcept {}

template <class FwdIt, class T, class... Ts>
constexpr void nmea_scanner_helper(FwdIt &beg, FwdIt end, T &t,
                                   Ts &... ts) noexcept {
  if (beg != end) {
    if (*beg == ',') {
      ++beg;
      if (*beg == ',') {
        return nmea_scanner_helper(beg, end, ts...);
      }
    }
    if (beg >= end || *beg == '*') {
      return;
    }
    extract(beg, end, t);
    return nmea_scanner_helper(beg, end, ts...);
  }
  __builtin_unreachable();
}

template <class... Ts>
constexpr void nmea_scanner(const std::string &format,
                            const std::string &sentence, Ts &... ts) noexcept;

template <class... Ts>
constexpr void nmea_scanner(const std::string &format,
                            const std::string &sentence, Ts &... ts) noexcept {
  auto beg = sentence.c_str() + format.size();
  auto end = sentence.c_str() + sentence.size() + 1;
  nmea_scanner_helper(beg, end, ts...);
}

template <class T, class... Ts>
constexpr void nmea_scanner(const std::string &format,
                            const std::string &sentence, T &t,
                            Ts &... ts) noexcept {
  auto beg = sentence.c_str() + format.size();
  auto end = sentence.c_str() + sentence.size() + 1;
  nmea_scanner_helper(beg, end, t, ts...);
}

template <class F, class stream_traits, class charT,
          class traits = std::char_traits<charT>>
auto &extractor(basic_gnss_istream<stream_traits, charT, traits> &strm,
                const std::string &prefix, F f) {
  while (strm) {
    std::string line;
    std::getline(strm, line, '\n');
    if (line.find(prefix) != std::string::npos) {
      f(line);
      return strm;
    }
  }
  return strm;
}
}

namespace skytraq {
class version final {
public:
  constexpr version() = default;

  std::string get_version_string() const {
    std::ostringstream str;
    str << kernel_version_x_ << "." << kernel_version_y_ << "."
        << kernel_version_z_;
    str << "-";
    str << odm_version_x_ << "." << odm_version_y_ << "." << odm_version_z_;
    str << "-";
    str << revision_dd_ << "." << revision_mm_ << "." << revision_yy_;
    return str.str();
  }

private:
  unsigned int kernel_version_x_{0};
  unsigned int kernel_version_y_{0};
  unsigned int kernel_version_z_{0};
  unsigned int odm_version_x_{0};
  unsigned int odm_version_y_{0};
  unsigned int odm_version_z_{0};
  unsigned int revision_dd_{0};
  unsigned int revision_mm_{0};
  unsigned int revision_yy_{0};

  template <class stream_traits, class charT, class traits>
  friend auto operator>>(basic_gnss_istream<stream_traits, charT, traits> &strm,
                         version &ver) -> decltype(strm);
};

class navigation_data final {
public:
  constexpr navigation_data() = default;

  constexpr int get_ecef_x() const noexcept { return ecef_x_; }
  constexpr int get_ecef_y() const noexcept { return ecef_y_; }
  constexpr int get_ecef_z() const noexcept { return ecef_z_; }
  constexpr int get_ecef_vx() const noexcept { return ecef_vx_; }
  constexpr int get_ecef_vy() const noexcept { return ecef_vy_; }
  constexpr int get_ecef_vz() const noexcept { return ecef_vz_; }
  constexpr unsigned int get_altitude() const noexcept { return altitude_; }
  constexpr fix_mode_skytraq get_fix_mode() const noexcept { return fix_mode_; }
  constexpr unsigned short get_gdop() const noexcept { return gdop_; }
  constexpr unsigned short get_hdop() const noexcept { return hdop_; }
  constexpr short get_gnss_week() const noexcept { return gnss_week_; }
  constexpr int get_latitude() const noexcept { return latitude_; }
  constexpr int get_longitude() const noexcept { return longitude_; }
  constexpr unsigned short get_pdop() const noexcept { return pdop_; }
  constexpr short get_satellites_in_view() const noexcept {
    return satellites_in_view_;
  }
  constexpr unsigned int get_separation() const noexcept { return diff_; }
  constexpr unsigned short get_tdop() const noexcept { return tdop_; }
  constexpr unsigned int get_time_of_week() const noexcept { return tow_; }
  constexpr float get_time_of_week_as_float() const noexcept {
    return tow_ / 100.0f;
  }
  constexpr unsigned short get_vdop() const noexcept { return vdop_; }

private:
  int ecef_x_{0};
  int ecef_y_{0};
  int ecef_z_{0};
  int ecef_vx_{0};
  int ecef_vy_{0};
  int ecef_vz_{0};
  int latitude_{0};
  int longitude_{0};
  unsigned int altitude_{0};
  unsigned int diff_{0};
  unsigned int tow_{0};
  short gnss_week_{0};
  short satellites_in_view_{0};
  unsigned short gdop_{0};
  unsigned short hdop_{0};
  unsigned short pdop_{0};
  unsigned short tdop_{0};
  unsigned short vdop_{0};
  fix_mode_skytraq fix_mode_{fix_mode_skytraq::invalid};

  template <class stream_traits, class charT, class traits>
  friend auto operator>>(basic_gnss_istream<stream_traits, charT, traits> &strm,
                         navigation_data &nd) -> decltype(strm);
};
}

template <class> class gnssgga final {
public:
  constexpr gnssgga() = default;

  constexpr float get_altitude() const noexcept { return altitude_; }
  constexpr float get_dgnss_age() const noexcept { return dgnss_age_; }
  constexpr short get_dgnss_sid() const noexcept { return dgnss_sid_; }
  constexpr gnss_quality_indicator get_fix_quality() const noexcept {
    return fix_quality_;
  }
  constexpr float get_hdop() const noexcept { return hdop_; }
  constexpr float get_latitude() const noexcept { return latitude_; }
  constexpr cardinal_point get_latitude_cardinal_pt() const noexcept {
    return lat_cardinal_pt_;
  }
  constexpr float get_longitude() const noexcept { return longitude_; }
  constexpr cardinal_point get_longitude_cardinal_pt() const noexcept {
    return lon_cardinal_pt_;
  }
  constexpr short get_satellites_in_use() const noexcept {
    return satellites_in_use_;
  }
  constexpr float get_separation() const noexcept { return diff_; }
  constexpr time_point get_utc() const noexcept { return utc_; }

private:
  time_point utc_;
  float latitude_{0.0f};
  float longitude_{0.0f};
  float hdop_{0.0f};
  float altitude_{0.0f};
  float diff_{0.0f};
  float dgnss_age_{0.0f};
  short dgnss_sid_{0};
  short satellites_in_use_{0};
  gnss_quality_indicator fix_quality_{gnss_quality_indicator::invalid};
  cardinal_point lat_cardinal_pt_{cardinal_point::north};
  cardinal_point lon_cardinal_pt_{cardinal_point::west};

  template <class stream_traits, class charT, class traits, class gnss>
  friend auto operator>>(basic_gnss_istream<stream_traits, charT, traits> &strm,
                         gnssgga<gnss> &gga) -> decltype(strm);
};

struct gpgga_traits {
  static const std::string value;
};

struct gngga_traits {
  static const std::string value;
};

using gpgga = gnssgga<gpgga_traits>;
using gngga = gnssgga<gngga_traits>;

template <class> class gnssgll final {
public:
  constexpr gnssgll() = default;

  constexpr float get_latitude() const noexcept { return latitude_; }
  constexpr cardinal_point get_latitude_cardinal_pt() const noexcept {
    return lat_cardinal_pt_;
  }
  constexpr float get_longitude() const noexcept { return longitude_; }
  constexpr cardinal_point get_longitude_cardinal_pt() const noexcept {
    return lon_cardinal_pt_;
  }
  constexpr fix_type get_mode() const noexcept { return mode_; }
  constexpr status get_status() const noexcept { return status_; }
  constexpr time_point get_utc() const noexcept { return utc_; }

private:
  time_point utc_;
  float latitude_{0.0f};
  float longitude_{0.0f};
  fix_type mode_{fix_type::not_valid};
  cardinal_point lat_cardinal_pt_{cardinal_point::north};
  cardinal_point lon_cardinal_pt_{cardinal_point::west};
  status status_{status::inactive};

  template <class stream_traits, class charT, class traits, class gnss>
  friend auto operator>>(basic_gnss_istream<stream_traits, charT, traits> &strm,
                         gnssgll<gnss> &gll) -> decltype(strm);
};

struct gpgll_traits {
  static const std::string value;
};

struct gngll_traits {
  static const std::string value;
};

using gpgll = gnssgll<gpgll_traits>;
using gngll = gnssgll<gngll_traits>;

template <class> class gnssgsa final {
public:
  constexpr gnssgsa() = default;

  constexpr fix_mode get_fix_mode() const noexcept { return mode_; }
  constexpr fix_geometry get_fix_geometry() const noexcept { return geometry_; }
  constexpr float get_hdop() const noexcept { return hdop_; }
  constexpr float get_pdop() const noexcept { return pdop_; }
  constexpr decltype(auto) get_prn() const noexcept { return sat_prn_; }
  constexpr float get_vdop() const noexcept { return vdop_; }

private:
  std::array<short, 12> sat_prn_{{0}};
  float hdop_{0.0f};
  float pdop_{0.0f};
  float vdop_{0.0f};
  fix_mode mode_{fix_mode::manual};
  fix_geometry geometry_{fix_geometry::invalid};

  template <class stream_traits, class charT, class traits, class gnss>
  friend auto operator>>(basic_gnss_istream<stream_traits, charT, traits> &strm,
                         gnssgsa<gnss> &gsa) -> decltype(strm);
};

struct bdgsa_traits {
  static const std::string value;
};

struct glgsa_traits {
  static const std::string value;
};

struct gngsa_traits {
  static const std::string value;
};

struct gpgsa_traits {
  static const std::string value;
};

using bdgsa = gnssgsa<bdgsa_traits>;
using glgsa = gnssgsa<glgsa_traits>;
using gngsa = gnssgsa<gngsa_traits>;
using gpgsa = gnssgsa<gpgsa_traits>;

template <class> class gnssgsv final {
public:
  class satellite_info {
  public:
    constexpr satellite_info() = default;

    constexpr short get_azimuth() const noexcept { return azimuth_; }
    constexpr short get_elevation() const noexcept { return elevation_; }
    constexpr short get_prn() const noexcept { return prn_; }
    constexpr short get_snr() const noexcept { return snr_; }

  private:
    short azimuth_{0};
    short elevation_{0};
    short prn_{0};
    short snr_{0};

    template <class stream_traits, class charT, class traits, class gnss>
    friend auto
    operator>>(basic_gnss_istream<stream_traits, charT, traits> &strm,
               gnssgsv<gnss> &gsv) -> decltype(strm);
  };

  constexpr gnssgsv() = default;

  constexpr short get_count() const noexcept { return count_; }
  constexpr short get_message_no() const noexcept { return message_no_; }
  constexpr short get_satellites_in_view() const noexcept {
    return satellites_in_view_;
  }
  constexpr satellite_info get_satellite_info(unsigned int i) const noexcept {
    return satellite_info_[i];
  }

private:
  std::array<satellite_info, 4> satellite_info_;
  short count_{0};
  short message_no_{-1};
  short satellites_in_view_{0};

  template <class stream_traits, class charT, class traits, class gnss>
  friend auto operator>>(basic_gnss_istream<stream_traits, charT, traits> &strm,
                         gnssgsv<gnss> &gsv) -> decltype(strm);
};

struct bdgsv_traits {
  static const std::string value;
};

struct gpgsv_traits {
  static const std::string value;
};

struct glgsv_traits {
  static const std::string value;
};

using bdgsv = gnssgsv<bdgsv_traits>;
using gpgsv = gnssgsv<gpgsv_traits>;
using glgsv = gnssgsv<glgsv_traits>;

template <class> class gnssrmc final {
public:
  constexpr gnssrmc() = default;

  std::string get_date() const noexcept {
    std::ostringstream str;
    str << std::put_time(&date_, "%d%m%y");
    return str.str();
  }
  constexpr float get_declination() const noexcept { return declination_; }
  constexpr cardinal_point get_declination_cardinal_pt() const noexcept {
    return declination_cardinal_pt_;
  }
  constexpr float get_direction() const noexcept { return direction_; }
  constexpr float get_latitude() const noexcept { return latitude_; }
  constexpr cardinal_point get_latitude_cardinal_pt() const noexcept {
    return lat_cardinal_pt_;
  }
  constexpr float get_longitude() const noexcept { return longitude_; }
  constexpr cardinal_point get_longitude_cardinal_pt() const noexcept {
    return lon_cardinal_pt_;
  }
  constexpr fix_type get_mode() const noexcept { return mode_; }
  constexpr float get_speed() const noexcept { return speed_; }
  constexpr status get_status() const noexcept { return status_; }
  constexpr time_point get_utc() const noexcept { return utc_; }

private:
  time_point utc_;
  struct tm date_ {};
  float latitude_{0.0f};
  float longitude_{0.0f};
  float speed_{0.0f};
  float direction_{0.0f};
  float declination_{0.0f};
  fix_type mode_{fix_type::not_valid};
  cardinal_point lat_cardinal_pt_{cardinal_point::north};
  cardinal_point lon_cardinal_pt_{cardinal_point::west};
  cardinal_point declination_cardinal_pt_{cardinal_point::west};
  status status_{status::inactive};

  template <class stream_traits, class charT, class traits, class gnss>
  friend auto operator>>(basic_gnss_istream<stream_traits, charT, traits> &strm,
                         gnssrmc<gnss> &rmc) -> decltype(strm);
};

struct gprmc_traits {
  static const std::string value;
};

struct gnrmc_traits {
  static const std::string value;
};

using gprmc = gnssrmc<gprmc_traits>;
using gnrmc = gnssrmc<gnrmc_traits>;

template <class> class gnssvtg final {
public:
  constexpr gnssvtg() = default;

  constexpr float get_direction() const noexcept { return direction_; }
  constexpr float get_declination() const noexcept { return declination_; }
  constexpr fix_type get_mode() const noexcept { return mode_; }
  constexpr float get_speed_knots() const noexcept { return speed_knots_; }
  constexpr float get_speed_kilometers() const noexcept {
    return speed_kilometers_;
  }

private:
  float direction_{0.0f};
  float declination_{0.0f};
  float speed_knots_{0.0f};
  float speed_kilometers_{0.0f};
  fix_type mode_{fix_type::not_valid};

  template <class stream_traits, class charT, class traits, class gnss>
  friend auto operator>>(basic_gnss_istream<stream_traits, charT, traits> &strm,
                         gnssvtg<gnss> &vtg) -> decltype(strm);
};

struct gpvtg_traits {
  static const std::string value;
};

struct gnvtg_traits {
  static const std::string value;
};

using gpvtg = gnssvtg<gpvtg_traits>;
using gnvtg = gnssvtg<gnvtg_traits>;

template <class> class gnsszda final {
public:
  constexpr gnsszda() = default;

  constexpr short get_day() const noexcept { return day_; }
  constexpr short get_month() const noexcept { return month_; }
  constexpr short get_tz_hours() const noexcept { return tz_hours_; }
  constexpr short get_tz_minutes() const noexcept { return tz_minutes_; }
  constexpr time_point get_utc() const noexcept { return utc_; }
  constexpr short get_year() const noexcept { return year_; }

private:
  time_point utc_;
  short day_{0};
  short month_{0};
  short tz_hours_{0};
  short tz_minutes_{0};
  short year_{0};

  template <class stream_traits, class charT, class traits, class gnss>
  friend auto operator>>(basic_gnss_istream<stream_traits, charT, traits> &strm,
                         gnsszda<gnss> &zda) -> decltype(strm);
};

struct gpzda_traits {
  static const std::string value;
};

struct gnzda_traits {
  static const std::string value;
};

using gpzda = gnsszda<gpzda_traits>;
using gnzda = gnsszda<gnzda_traits>;

template <class stream_traits, class charT, class traits, class gnss>
auto operator>>(basic_gnss_istream<stream_traits, charT, traits> &strm,
                gnssgga<gnss> &gga) -> decltype(strm) {
  return detail::extractor(strm, gnss::value, [&gga](const std::string &line) {
    char alt_unit;
    char diff_unit;
    detail::nmea_scanner(
        gnss::value, line, gga.utc_, gga.latitude_, gga.lat_cardinal_pt_,
        gga.longitude_, gga.lon_cardinal_pt_, gga.fix_quality_,
        gga.satellites_in_use_, gga.hdop_, gga.altitude_, alt_unit, gga.diff_,
        diff_unit, gga.dgnss_age_, gga.dgnss_sid_);

  });
}

template <class stream_traits, class charT, class traits, class gnss>
auto operator>>(basic_gnss_istream<stream_traits, charT, traits> &strm,
                gnssgll<gnss> &gll) -> decltype(strm) {
  return detail::extractor(strm, gnss::value, [&gll](const std::string &line) {
    detail::nmea_scanner(gnss::value, line, gll.latitude_, gll.lat_cardinal_pt_,
                         gll.longitude_, gll.lon_cardinal_pt_, gll.utc_,
                         gll.status_, gll.mode_);
  });
}

template <class stream_traits, class charT, class traits, class gnss>
auto operator>>(basic_gnss_istream<stream_traits, charT, traits> &strm,
                gnssrmc<gnss> &rmc) -> decltype(strm) {
  return detail::extractor(strm, gnss::value, [&rmc](const std::string &line) {
    detail::nmea_scanner(gnss::value, line, rmc.utc_, rmc.status_,
                         rmc.latitude_, rmc.lat_cardinal_pt_, rmc.longitude_,
                         rmc.lon_cardinal_pt_, rmc.speed_, rmc.direction_,
                         rmc.date_, rmc.declination_,
                         rmc.declination_cardinal_pt_, rmc.mode_);
  });
}

template <class stream_traits, class charT, class traits, class gnss>
auto operator>>(basic_gnss_istream<stream_traits, charT, traits> &strm,
                gnssvtg<gnss> &vtg) -> decltype(strm) {
  char direction_t;
  char declination_t;
  char knots;
  char kilometers;
  return detail::extractor(
      strm, gnss::value,
      [&vtg, &direction_t, &declination_t, &knots, &kilometers](
          const std::string &line) {
        detail::nmea_scanner(gnss::value, line, vtg.direction_, direction_t,
                             vtg.declination_, declination_t, vtg.speed_knots_,
                             knots, vtg.speed_kilometers_, kilometers,
                             vtg.mode_);
      });
}

template <class stream_traits, class charT, class traits, class gnss>
auto operator>>(basic_gnss_istream<stream_traits, charT, traits> &strm,
                gnssgsa<gnss> &gsa) -> decltype(strm) {
  return detail::extractor(strm, gnss::value, [&gsa](const std::string &line) {
    detail::nmea_scanner(gnss::value, line, gsa.mode_, gsa.geometry_,
                         gsa.sat_prn_[0], gsa.sat_prn_[1], gsa.sat_prn_[2],
                         gsa.sat_prn_[3], gsa.sat_prn_[4], gsa.sat_prn_[5],
                         gsa.sat_prn_[6], gsa.sat_prn_[7], gsa.sat_prn_[8],
                         gsa.sat_prn_[9], gsa.sat_prn_[10], gsa.sat_prn_[11],
                         gsa.pdop_, gsa.hdop_, gsa.vdop_);
  });
}

template <class stream_traits, class charT, class traits, class gnss>
auto operator>>(basic_gnss_istream<stream_traits, charT, traits> &strm,
                gnssgsv<gnss> &gsv) -> decltype(strm) {
  return detail::extractor(strm, gnss::value, [&gsv](const std::string &line) {
    detail::nmea_scanner(
        gnss::value, line, gsv.count_, gsv.message_no_, gsv.satellites_in_view_,
        gsv.satellite_info_[0].prn_, gsv.satellite_info_[0].elevation_,
        gsv.satellite_info_[0].azimuth_, gsv.satellite_info_[0].snr_,
        gsv.satellite_info_[1].prn_, gsv.satellite_info_[1].elevation_,
        gsv.satellite_info_[1].azimuth_, gsv.satellite_info_[1].snr_,
        gsv.satellite_info_[2].prn_, gsv.satellite_info_[2].elevation_,
        gsv.satellite_info_[2].azimuth_, gsv.satellite_info_[2].snr_,
        gsv.satellite_info_[3].prn_, gsv.satellite_info_[3].elevation_,
        gsv.satellite_info_[3].azimuth_, gsv.satellite_info_[3].snr_);
  });
}

template <class stream_traits, class charT, class traits, class gnss>
auto operator>>(basic_gnss_istream<stream_traits, charT, traits> &strm,
                gnsszda<gnss> &zda) -> decltype(strm) {
  return detail::extractor(strm, gnss::value, [&zda](const std::string &line) {
    detail::nmea_scanner(gnss::value, line, zda.utc_, zda.day_, zda.month_,
                         zda.year_, zda.tz_hours_, zda.tz_minutes_);
  });
}

template <class stream_traits, class charT, class traits>
auto skytraq::operator>>(basic_gnss_istream<stream_traits, charT, traits> &strm,
                         skytraq::version &ver) -> decltype(strm) {
  while (strm) {
    std::string line;
    std::getline(strm, line, '\r');
    strm.ignore(1, '\n');
    if (line[0] == 0xA0 && line[1] == 0xA1 && line[4] == 0x80) {
      ver.kernel_version_x_ = static_cast<unsigned int>(line[7]);
      ver.kernel_version_y_ = static_cast<unsigned int>(line[8]);
      ver.kernel_version_z_ = static_cast<unsigned int>(line[9]);
      ver.odm_version_x_ = static_cast<unsigned int>(line[11]);
      ver.odm_version_y_ = static_cast<unsigned int>(line[12]);
      ver.odm_version_z_ = static_cast<unsigned int>(line[13]);
      ver.revision_yy_ = static_cast<unsigned int>(line[15]);
      ver.revision_mm_ = static_cast<unsigned int>(line[16]);
      ver.revision_dd_ = static_cast<unsigned int>(line[17]);
      break;
    }
  }
  return strm;
}

template <class stream_traits, class charT, class traits>
auto skytraq::operator>>(basic_gnss_istream<stream_traits, charT, traits> &strm,
                         skytraq::navigation_data &nd) -> decltype(strm) {
  while (strm) {
    std::string line;
    std::getline(strm, line, '\r');
    strm.ignore(1, '\n');
    if (line[0] == 0xA0 && line[1] == 0xA1 && line[4] == 0xA8) {
      nd.fix_mode_ = fix_mode_skytraq::_3D;
      nd.satellites_in_view_ = line[6];
      nd.gnss_week_ = static_cast<decltype(nd.gnss_week_)>(
          detail::shifter(line[7], line[8]));
      nd.tow_ = static_cast<decltype(nd.tow_)>(
          detail::shifter(line[9], line[10], line[11], line[12]));
      nd.latitude_ = detail::shifter(line[13], line[14], line[15], line[16]);
      nd.longitude_ = detail::shifter(line[17], line[18], line[19], line[20]);
      nd.altitude_ = static_cast<decltype(nd.altitude_)>(
          detail::shifter(line[21], line[22], line[23], line[24]));
      nd.diff_ = static_cast<decltype(nd.altitude_)>(
          detail::shifter(line[25], line[26], line[27], line[28]));
      nd.gdop_ =
          static_cast<decltype(nd.gdop_)>(detail::shifter(line[29], line[30]));
      nd.pdop_ =
          static_cast<decltype(nd.pdop_)>(detail::shifter(line[31], line[32]));
      nd.hdop_ =
          static_cast<decltype(nd.hdop_)>(detail::shifter(line[33], line[34]));
      nd.vdop_ =
          static_cast<decltype(nd.vdop_)>(detail::shifter(line[35], line[36]));
      nd.tdop_ =
          static_cast<decltype(nd.tdop_)>(detail::shifter(line[37], line[38]));
      nd.ecef_x_ = detail::shifter(line[39], line[40], line[41], line[42]);
      nd.ecef_y_ = detail::shifter(line[43], line[44], line[45], line[46]);
      nd.ecef_z_ = detail::shifter(line[47], line[48], line[49], line[50]);
      nd.ecef_vx_ = detail::shifter(line[51], line[52], line[53], line[54]);
      nd.ecef_vy_ = detail::shifter(line[55], line[56], line[57], line[58]);
      nd.ecef_vz_ = detail::shifter(line[59], line[60], line[61], line[62]);
      break;
    }
  }
  return strm;
}
}
}
