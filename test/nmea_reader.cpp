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

#ifdef __clang__
#pragma clang diagnostic ignored "-Wglobal-constructors"
#endif

#define STRINGIFY(x) #x
#define EXPAND_MACRO(x) STRINGIFY(x)

#include <fstream>
#include <iostream>
#include <iterator>
#include <vector>

#define BOOST_TEST_MODULE nmea_reader
#include <boost/test/included/unit_test.hpp>

#include <riot/navigation.hpp>

namespace nav = riot::navigation;

struct device_stream_traits final : public nav::basic_stream_traits {};

using gnss_istream = nav::basic_gnss_istream<device_stream_traits, char>;

namespace {
static std::vector<char> erase_newline(std::vector<char> &vec) {
  vec.erase(std::remove_if(vec.begin(), vec.end(), [](auto c) {
              return c == '\n';
            }), vec.end());
  return vec;
}

static std::vector<char> reference(std::ifstream &&fstrm) {
  std::vector<char> original{std::istream_iterator<char>(fstrm),
                             std::istream_iterator<char>()};
  return erase_newline(original);
}
}

BOOST_AUTO_TEST_SUITE(nmea_reader)

BOOST_AUTO_TEST_CASE(check_exception) {
  try {
    gnss_istream strm{"nonexistent.txt"};
  } catch (const std::exception &) {
    BOOST_CHECK(true);
    return;
  }
  __builtin_unreachable();
}

BOOST_AUTO_TEST_CASE(check_get) {
  gnss_istream strm{EXPAND_MACRO(REFERENCE_DATA_DIR) "void.txt"};
  std::vector<char> gnss;
  char c;
  while (strm.get(c)) {
    if (c != '\n') {
      gnss.push_back(c);
    }
  }
  BOOST_CHECK(reference(std::ifstream{
                  EXPAND_MACRO(REFERENCE_DATA_DIR) "void.txt"}) == gnss);
}

BOOST_AUTO_TEST_CASE(check_getline) {
  gnss_istream strm(EXPAND_MACRO(REFERENCE_DATA_DIR) "notvoid.txt");
  std::vector<char> gnss;
  std::string line;
  while (std::getline(strm, line)) {
    gnss.insert(gnss.end(), line.begin(), line.end());
  }
  BOOST_CHECK(reference(std::ifstream{EXPAND_MACRO(
                  REFERENCE_DATA_DIR) "notvoid.txt"}) == erase_newline(gnss));
}

BOOST_AUTO_TEST_CASE(check_operator) {
  gnss_istream strm(EXPAND_MACRO(REFERENCE_DATA_DIR) "notvoid.txt");
  std::vector<char> gnss;
  std::string line;
  while (!strm.eof()) {
    strm >> line;
    gnss.insert(gnss.end(), line.begin(), line.end());
    line.clear();
  }
  BOOST_CHECK(reference(std::ifstream{EXPAND_MACRO(
                  REFERENCE_DATA_DIR) "notvoid.txt"}) == erase_newline(gnss));
}

BOOST_AUTO_TEST_CASE(check_iterator_copy) {
  gnss_istream strm(EXPAND_MACRO(REFERENCE_DATA_DIR) "notvoid.txt");
  strm >> std::noskipws;
  std::vector<char> gnss;
  std::copy(std::istream_iterator<char>(strm), std::istream_iterator<char>(),
            std::back_inserter(gnss));
  BOOST_CHECK(reference(std::ifstream{EXPAND_MACRO(
                  REFERENCE_DATA_DIR) "notvoid.txt"}) == erase_newline(gnss));
}

BOOST_AUTO_TEST_CASE(tie) {
  gnss_istream strm(EXPAND_MACRO(REFERENCE_DATA_DIR) "notvoid.txt");
  strm.tie(&std::cout);
  std::vector<char> gnss;
  std::copy(std::istream_iterator<char>(strm), std::istream_iterator<char>(),
            std::back_inserter(gnss));
  BOOST_CHECK(reference(std::ifstream{EXPAND_MACRO(
                  REFERENCE_DATA_DIR) "notvoid.txt"}) == erase_newline(gnss));
}

BOOST_AUTO_TEST_CASE(basic_skytraq) {
  gnss_istream strm(EXPAND_MACRO(REFERENCE_DATA_DIR) "skytraq.release.bin");
  std::vector<char> gnss;
  std::copy(std::istream_iterator<char>(strm), std::istream_iterator<char>(),
            std::back_inserter(gnss));
  BOOST_CHECK(reference(std::ifstream{
                  EXPAND_MACRO(REFERENCE_DATA_DIR) "skytraq.release.bin"}) ==
              erase_newline(gnss));
}

BOOST_AUTO_TEST_CASE(skytraq_release) {
  gnss_istream strm(EXPAND_MACRO(REFERENCE_DATA_DIR) "skytraq.release.bin");
  nav::skytraq::version version;
  strm >> version;
  BOOST_CHECK(version.get_version_string() == "1.6.10-1.10.23-10.7.12");
}

BOOST_AUTO_TEST_CASE(gngga) {
  using namespace std::chrono_literals;
  auto current_time_point =
      nav::time_point(std::chrono::hours(7) + std::chrono::minutes(10) +
                      std::chrono::seconds(55) + std::chrono::milliseconds(0));
  gnss_istream strm(EXPAND_MACRO(REFERENCE_DATA_DIR) "glonass.txt");
  while (strm) {
    nav::gngga gga;
    strm >> gga;
    if (strm) {
      BOOST_CHECK(gga.get_utc() == current_time_point);
      BOOST_CHECK(gga.get_fix_quality() == nav::gnss_quality_indicator::fix);
      BOOST_CHECK(gga.get_satellites_in_use() == 8 ||
                  gga.get_satellites_in_use() == 9);
      current_time_point += 1s;
    }
  }
}

BOOST_AUTO_TEST_CASE(gpgga) {
  using namespace std::chrono_literals;
  auto current_time_point = nav::time_point(
      std::chrono::hours(16) + std::chrono::minutes(0) +
      std::chrono::seconds(33) + std::chrono::milliseconds(184));
  gnss_istream strm(EXPAND_MACRO(REFERENCE_DATA_DIR) "notvoid.txt");
  while (strm) {
    nav::gpgga gga;
    strm >> gga;
    if (strm) {
      BOOST_CHECK(gga.get_utc() == current_time_point);
      BOOST_CHECK(gga.get_fix_quality() == nav::gnss_quality_indicator::fix);
      BOOST_CHECK(gga.get_satellites_in_use() == 4);
      current_time_point += 1s;
    }
  }
}

BOOST_AUTO_TEST_CASE(gngll) {
  using namespace std::chrono_literals;
  auto current_time_point =
      nav::time_point(std::chrono::hours(7) + std::chrono::minutes(10) +
                      std::chrono::seconds(55) + std::chrono::milliseconds(0));
  gnss_istream strm(EXPAND_MACRO(REFERENCE_DATA_DIR) "glonass.txt");
  while (strm) {
    nav::gngll gll;
    strm >> gll;
    if (strm) {
      BOOST_CHECK(gll.get_utc() == current_time_point);
      BOOST_CHECK(fabsf(gll.get_latitude() - 5127.2212f) < 1e-6f);
      BOOST_CHECK(fabsf(gll.get_longitude() - 58.9794f) < 1e-6f);
      current_time_point += 1s;
    }
  }
}

BOOST_AUTO_TEST_CASE(gnrmc) {
  using namespace std::chrono_literals;
  auto current_time_point =
      nav::time_point(std::chrono::hours(7) + std::chrono::minutes(10) +
                      std::chrono::seconds(55) + std::chrono::milliseconds(0));
  gnss_istream strm(EXPAND_MACRO(REFERENCE_DATA_DIR) "glonass.txt");
  while (strm) {
    nav::gnrmc rmc;
    strm >> rmc;
    if (strm) {
      BOOST_CHECK(rmc.get_utc() == current_time_point);
      BOOST_CHECK(rmc.get_date() == "011015");
      current_time_point += 1s;
    }
  }
}

BOOST_AUTO_TEST_CASE(gprmc) {
  using namespace std::chrono_literals;
  auto current_time_point = nav::time_point(
      std::chrono::hours(16) + std::chrono::minutes(0) +
      std::chrono::seconds(33) + std::chrono::milliseconds(184));
  gnss_istream strm(EXPAND_MACRO(REFERENCE_DATA_DIR) "notvoid.txt");
  while (strm) {
    nav::gprmc rmc;
    strm >> rmc;
    if (strm) {
      BOOST_CHECK(rmc.get_utc() == current_time_point);
      BOOST_CHECK(rmc.get_date() == "080215");
      current_time_point += 1s;
    }
  }
}

BOOST_AUTO_TEST_CASE(gnvtg) {
  gnss_istream strm(EXPAND_MACRO(REFERENCE_DATA_DIR) "glonass.txt");
  while (strm) {
    nav::gnvtg vtg;
    strm >> vtg;
    if (strm) {
      BOOST_CHECK(fabsf(vtg.get_direction() - 195.9f) < 1e-6f);
      BOOST_CHECK(vtg.get_speed_knots() == 0.0f);
      BOOST_CHECK(vtg.get_speed_kilometers() == 0.0f);
      BOOST_CHECK(vtg.get_mode() == nav::fix_type::autonomous);
    }
  }
}

BOOST_AUTO_TEST_CASE(gpvtg) {
  gnss_istream strm(EXPAND_MACRO(REFERENCE_DATA_DIR) "notvoid.txt");
  while (strm) {
    nav::gpvtg vtg;
    strm >> vtg;
    if (strm) {
      BOOST_CHECK(fabsf(vtg.get_direction() - 174.6f) < 1e-6f);
      BOOST_CHECK(vtg.get_speed_knots() == 0.0f);
      BOOST_CHECK(vtg.get_speed_kilometers() == 0.0f);
      BOOST_CHECK(vtg.get_mode() == nav::fix_type::autonomous);
    }
  }
}

BOOST_AUTO_TEST_CASE(gngsa) {
  gnss_istream strm(EXPAND_MACRO(REFERENCE_DATA_DIR) "glonass.txt");
  while (strm) {
    nav::gngsa gsa;
    strm >> gsa;
    if (strm) {
      BOOST_CHECK(gsa.get_fix_mode() == nav::fix_mode::autonomous);
      BOOST_CHECK(gsa.get_fix_geometry() == nav::fix_geometry::_3D);
      const auto prn = gsa.get_prn();
      BOOST_CHECK(prn[0] == 12);
      BOOST_CHECK(prn[1] == 31);
      BOOST_CHECK(prn[2] == 25);
      BOOST_CHECK(prn[3] == 29);
      BOOST_CHECK(prn[4] == 26);
      BOOST_CHECK(prn[5] == 16);
      BOOST_CHECK(fabsf(gsa.get_hdop() - 1.8f) < 1e-6f);
      BOOST_CHECK(fabsf(gsa.get_pdop() - 2.2f) < 1e-6f);
      BOOST_CHECK(fabsf(gsa.get_vdop() - 1.2f) < 1e-6f);
      break;
    }
  }
}

BOOST_AUTO_TEST_CASE(gpgsa) {
  gnss_istream strm(EXPAND_MACRO(REFERENCE_DATA_DIR) "notvoid.txt");
  while (strm) {
    nav::gpgsa gsa;
    strm >> gsa;
    if (strm) {
      BOOST_CHECK(gsa.get_fix_mode() == nav::fix_mode::autonomous);
      BOOST_CHECK(gsa.get_fix_geometry() == nav::fix_geometry::_3D);
      const auto prn = gsa.get_prn();
      BOOST_CHECK(prn[0] == 15);
      BOOST_CHECK(prn[1] == 13);
      BOOST_CHECK(prn[2] == 21);
      BOOST_CHECK(prn[3] == 18);
      BOOST_CHECK(prn[4] == 0);
      BOOST_CHECK(fabsf(gsa.get_hdop() - 4.0f) < 1e-6f);
      BOOST_CHECK(fabsf(gsa.get_pdop() - 5.4f) < 1e-6f);
      BOOST_CHECK(fabsf(gsa.get_vdop() - 3.5f) < 1e-6f);
    }
  }
}

BOOST_AUTO_TEST_CASE(bdgsv) {
  gnss_istream strm(EXPAND_MACRO(REFERENCE_DATA_DIR) "beidou.txt");
  if (strm) {
    nav::bdgsv gsv_1;
    strm >> gsv_1;
    BOOST_CHECK(gsv_1.get_count() == 1);
    BOOST_CHECK(gsv_1.get_message_no() == 1);
    BOOST_CHECK(gsv_1.get_satellites_in_view() == 3);

    auto check_satellite_data = [](const auto &gsv, auto i, auto prn,
                                   auto elevation, auto azimuth, auto snr) {
      auto satellite = gsv.get_satellite_info(i);
      BOOST_CHECK(satellite.get_prn() == prn);
      BOOST_CHECK(satellite.get_elevation() == elevation);
      BOOST_CHECK(satellite.get_azimuth() == azimuth);
      BOOST_CHECK(satellite.get_snr() == snr);
    };

    check_satellite_data(gsv_1, 0u, 211, 66, 295, 32);
    check_satellite_data(gsv_1, 1u, 209, 27, 45, 0);
    check_satellite_data(gsv_1, 2u, 202, 0, 0, 0);
  }
}

BOOST_AUTO_TEST_CASE(gpgsv) {
  gnss_istream strm(EXPAND_MACRO(REFERENCE_DATA_DIR) "notvoid.txt");
  if (strm) {
    nav::gpgsv gsv_1;
    nav::gpgsv gsv_2;
    strm >> gsv_1;
    strm >> gsv_2;
    BOOST_CHECK(gsv_1.get_count() == 2);
    BOOST_CHECK(gsv_1.get_message_no() == 1);
    BOOST_CHECK(gsv_1.get_satellites_in_view() == 7);

    auto check_satellite_data = [](const auto &gsv, auto i, auto prn,
                                   auto elevation, auto azimuth, auto snr) {
      auto satellite = gsv.get_satellite_info(i);
      BOOST_CHECK(satellite.get_prn() == prn);
      BOOST_CHECK(satellite.get_elevation() == elevation);
      BOOST_CHECK(satellite.get_azimuth() == azimuth);
      BOOST_CHECK(satellite.get_snr() == snr);
    };

    check_satellite_data(gsv_1, 0u, 13, 81, 101, 29);
    check_satellite_data(gsv_1, 1u, 15, 56, 291, 27);
    check_satellite_data(gsv_1, 2u, 18, 17, 318, 32);
    check_satellite_data(gsv_1, 3u, 21, 10, 291, 18);

    BOOST_CHECK(gsv_2.get_count() == 2);
    BOOST_CHECK(gsv_2.get_message_no() == 2);
    BOOST_CHECK(gsv_2.get_satellites_in_view() == 7);

    check_satellite_data(gsv_2, 0u, 19, 4, 11, 0);
    check_satellite_data(gsv_2, 1u, 17, 3, 120, 0);
    check_satellite_data(gsv_2, 2u, 28, 0, 0, 24);
    check_satellite_data(gsv_2, 3u, 0, 0, 0, 0);
  }
}

BOOST_AUTO_TEST_CASE(gnzda) {
  using namespace std::chrono_literals;
  auto current_time_point =
      nav::time_point(std::chrono::hours(7) + std::chrono::minutes(10) +
                      std::chrono::seconds(55) + std::chrono::milliseconds(0));
  gnss_istream strm(EXPAND_MACRO(REFERENCE_DATA_DIR) "glonass.txt");
  while (strm) {
    nav::gnzda zda;
    strm >> zda;
    if (strm) {
      BOOST_CHECK(zda.get_utc() == current_time_point);
      BOOST_CHECK(zda.get_day() == 01);
      BOOST_CHECK(zda.get_month() == 10);
      BOOST_CHECK(zda.get_year() == 2015);
      BOOST_CHECK(zda.get_tz_hours() == 0);
      BOOST_CHECK(zda.get_tz_minutes() == 0);
      current_time_point += 1s;
    }
  }
}

BOOST_AUTO_TEST_CASE(navigation_data) {
  gnss_istream strm(EXPAND_MACRO(REFERENCE_DATA_DIR) "skytraq.bin");
  nav::skytraq::navigation_data nd;
  strm >> nd;
  BOOST_CHECK(nd.get_gnss_week() == 1863);
  BOOST_CHECK(nd.get_time_of_week() == 22630277);
  BOOST_CHECK(fabsf(nd.get_time_of_week_as_float() - 226302.77f) < 1e-1f);
}

BOOST_AUTO_TEST_SUITE_END()
