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

#ifdef __cplusplus
extern "C" {
#endif

#include <errno.h>
#include <fcntl.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <termios.h>
#include <unistd.h>

#ifdef __cplusplus
}
#endif

#include <cstring>

#include "exception.hpp"
#include "navigation.hpp"

using namespace std::literals::string_literals;

namespace riot {
namespace navigation {

namespace detail {
namespace {

#ifdef __clang__
#pragma clang diagnostic push
#pragma clang diagnostic ignored "-Wglobal-constructors"
#pragma clang diagnostic ignored "-Wexit-time-destructors"
#endif

// mt33xx
static const std::string PMTK_API_Q_SBAS_ENABLED = "$PMTK413*34C\r\n";
static const std::string PMTK_API_SET_DGPS_MODE_NONE = "$PMTK301,0*2C\r\n";
static const std::string PMTK_API_SET_DGPS_MODE_RTCM = "$PMTK301,1*2D\r\n";
static const std::string PMTK_API_SET_DGPS_MODE_WAAS = "$PMTK301,2*2E\r\n";
static const std::string PMTK_API_SET_SBAS_DISABLED = "$PMTK313,0*2F\r\n";
static const std::string PMTK_API_SET_SBAS_ENABLED = "$PMTK313,1*2E\r\n";
static const std::string PMTK_API_SET_SBAS_MODE_INTEGRITY = "$PMTK319,1*24\r\n";
static const std::string PMTK_API_SET_SBAS_MODE_TESTING = "$PMTK319,0*25\r\n";
static const std::string PMTK_CMD_AIC_MODE_DISABLE = "$PMTK286,0*22\r\n";
static const std::string PMTK_CMD_AIC_MODE_ENABLE = "$PMTK286,1*23\r\n";
static const std::string PMTK_CMD_COLD_START = "$PMTK103*30\r\n";
static const std::string PMTK_CMD_EASY_QUERY = "$PMTK869,0*29\r\n";
static const std::string PMTK_CMD_EASY_DISABLE = "$PMTK869,1,0*34\r\n";
static const std::string PMTK_CMD_EASY_ENABLE = "$PMTK869,1,1*35\r\n";
static const std::string PMTK_CMD_FULL_COLD_START = "$PMTK104*37\r\n";
static const std::string PMTK_CMD_HOT_START = "$PMTK101*32\r\n";
static const std::string PMTK_CMD_STANDBY_MODE = "$PMTK161,0*28\r\n";
static const std::string PMTK_CMD_WARM_START = "$PMTK102*31\r\n";
static const std::string PMTK_DT_SBAS_DISABLE = "$PMTK513,0*29\r\n";
static const std::string PMTK_DT_SBAS_ENABLE = "$PMTK513,1*28\r\n";
static const std::string PMTK_Q_NAV_THRESHOLD = "$PMTK447*35\r\n";
static const std::string PMTK_Q_RELEASE = "$PMTK605*31\r\n";
static const std::string PMTK_SET_NAV_SPEED_0 = "$PMTK397,0*23\r\n";
static const std::string PMTK_SET_NAV_SPEED_0_2 = "$PMTK397,0.2*3F\r\n";
static const std::string PMTK_SET_NAV_SPEED_0_4 = "$PMTK397,0.4*39\r\n";
static const std::string PMTK_SET_NAV_SPEED_0_6 = "$PMTK397,0.6*3B\r\n";
static const std::string PMTK_SET_NAV_SPEED_0_8 = "$PMTK397,0.8*35\r\n";
static const std::string PMTK_SET_NAV_SPEED_1_0 = "$PMTK397,1.0*3C\r\n";
static const std::string PMTK_SET_NAV_SPEED_2_0 = "$PMTK397,2.0*3F\r\n";
static const std::string PMTK_SET_NMEA_BAUD_RATE_4800 = "$PMTK251,4800*14\r\n";
static const std::string PMTK_SET_NMEA_BAUD_RATE_9600 = "$PMTK251,9600*17\r\n";
static const std::string PMTK_SET_NMEA_BAUD_RATE_14400 =
    "$PMTK251,14400*29\r\n";
static const std::string PMTK_SET_NMEA_BAUD_RATE_19200 =
    "$PMTK251,19200*22\r\n";
static const std::string PMTK_SET_NMEA_BAUD_RATE_38400 =
    "$PMTK251,38400*27\r\n";
static const std::string PMTK_SET_NMEA_BAUD_RATE_57600 =
    "$PMTK251,57600*2C\r\n";
static const std::string PMTK_SET_NMEA_BAUD_RATE_115200 =
    "$PMTK251,115200*1F\r\n";
static const std::string PMTK_SET_NMEA_BAUD_RATE_DEFAULT = "$PMTK251,0*28\r\n";
static const std::string PMTK_SET_NMEA_OUTPUT_DEFAULT = "$PMTK314,-1*04\r\n";
static const std::string PMTK_SET_NMEA_OUTPUT_OFF =
    "$PMTK314,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0*28\r\n";
static const std::string PMTK_SET_NMEA_OUTPUT_RMCONLY =
    "$PMTK314,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0*29\r\n";
static const std::string PMTK_SET_NMEA_UPDATERATE_1HZ = "$PMTK220,1000*1F\r\n";
static const std::string PMTK_SET_NMEA_UPDATERATE_5HZ = "$PMTK220,200*2C\r\n";
static const std::string PMTK_SET_NMEA_UPDATERATE_10HZ = "$PMTK220,100*2F\r\n";

// SkyTraq
static const std::string SKYTRAQ_Q_CRC =
    "\xA0\xA1\x00\x02\x03\x01\x02\x0D\x0A"s;
static const std::string SKYTRAQ_Q_EPHEMERIS =
    "\xA0\xA1\x00\x02\x30\x00\x30\x0D\x0A"s;
static const std::string SKYTRAQ_Q_NAVIGATION_MODE =
    "\xA0\xA1\x00\x01\x3D\x3D\x0D\x0A"s;
static const std::string SKYTRAQ_Q_UPDATERATE =
    "\xA0\xA1\x00\x01\x10\x10\x0D\x0A"s;
static const std::string SKYTRAQ_Q_RELEASE =
    "\xA0\xA1\x00\x02\x02\x01\x03\x0D\x0A"s;
static const std::string SKYTRAQ_Q_WAAS = "\xA0\xA1\x00\x01\x38\x38\x0D\x0A"s;
static const std::string SKYTRAQ_SET_BAUD_RATE_4800 =
    "\xA0\xA1\x00\x04\x05\x00\x00\x00\x05\x0D\x0A"s;
static const std::string SKYTRAQ_SET_BAUD_RATE_9600 =
    "\xA0\xA1\x00\x04\x05\x00\x00\x01\x04\x0D\x0A"s;
static const std::string SKYTRAQ_SET_BAUD_RATE_19200 =
    "\xA0\xA1\x00\x04\x05\x00\x00\x02\x07\x0D\x0A"s;
static const std::string SKYTRAQ_SET_BAUD_RATE_38400 =
    "\xA0\xA1\x00\x04\x05\x00\x00\x03\x06\x0D\x0A"s;
static const std::string SKYTRAQ_SET_BAUD_RATE_57600 =
    "\xA0\xA1\x00\x04\x05\x00\x00\x04\x01\x0D\x0A"s;
static const std::string SKYTRAQ_SET_BAUD_RATE_115200 =
    "\xA0\xA1\x00\x04\x05\x00\x00\x05\x00\x0D\x0A"s;
static const std::string SKYTRAQ_SET_DISABLE_WAAS =
    "\xA0\xA1\x00\x03\x37\x00\x00\x37\x0D\x0A"s;
static const std::string SKYTRAQ_SET_ENABLE_WAAS =
    "\xA0\xA1\x00\x03\x37\x01\x00\x36\x0D\x0A"s;
static const std::string SKYTRAQ_SET_FACTORY_DEFAULTS =
    "\xA0\xA1\x00\x02\x04\x01\x05\x0D\x0A"s;
static const std::string SKYTRAQ_SET_NAVIGATION_MODE_CAR =
    "\xA0\xA1\x00\x03\x3C\x00\x00\x3C\x0D\x0A"s;
static const std::string SKYTRAQ_SET_NAVIGATION_MODE_PEDESTRIAN =
    "\xA0\xA1\x00\x03\x3C\x01\x00\x3D\x0D\x0A"s;
static const std::string SKYTRAQ_SET_NMEA_OUTPUT_ALL =
    "\xA0\xA1\x00\x09\x08\x01\x01\x01\x01\x01\x01\x01\x00\x09\x0D\x0A"s;
static const std::string SKYTRAQ_SET_NMEA_OUTPUT_RMCONLY =
    "\xA0\xA1\x00\x09\x08\x00\x00\x00\x00\x01\x00\x00\x00\x09\x0D\x0A"s;
static const std::string SKYTRAQ_SET_OUTPUT_BINARY =
    "\xA0\xA1\x00\x03\x09\x02\x00\x0B\x0D\x0A"s;
static const std::string SKYTRAQ_SET_OUTPUT_OFF =
    "\xA0\xA1\x00\x03\x09\x00\x00\x09\x0D\x0A"s;
static const std::string SKYTRAQ_SET_OUTPUT_NMEA =
    "\xA0\xA1\x00\x03\x09\x01\x00\x08\x0D\x0A"s;
static const std::string SKYTRAQ_SET_POWER_MODE_NORMAL =
    "\xA0\xA1\x00\x03\x0C\x00\x00\x0C\x0D\x0A"s;
static const std::string SKYTRAQ_SET_POWER_MODE_SAVER =
    "\xA0\xA1\x00\x03\x0C\x01\x00\x0D\x0D\x0A"s;
static const std::string SKYTRAQ_SET_UPDATERATE_1HZ =
    "\xA0\xA1\x00\x03\x0E\x01\x00\x0F\x0D\x0A"s;
static const std::string SKYTRAQ_SET_UPDATERATE_2HZ =
    "\xA0\xA1\x00\x03\x0E\x02\x00\x0C\x0D\x0A"s;
static const std::string SKYTRAQ_SET_UPDATERATE_4HZ =
    "\xA0\xA1\x00\x03\x0E\x04\x00\x0A\x0D\x0A"s;
static const std::string SKYTRAQ_SET_UPDATERATE_5HZ =
    "\xA0\xA1\x00\x03\x0E\x05\x00\x0B\x0D\x0A"s;
static const std::string SKYTRAQ_SET_UPDATERATE_8HZ =
    "\xA0\xA1\x00\x03\x0E\x08\x00\x06\x0D\x0A"s;
static const std::string SKYTRAQ_SET_UPDATERATE_10HZ =
    "\xA0\xA1\x00\x03\x0E\x0A\x00\x04\x0D\x0A"s;

#ifdef __clang__
#pragma clang diagnostic pop
#endif

static bool check_nmea_sentence_checksum(const char *buffer) {
  auto to_hex = [](auto c) {
    if (c <= '9') {
      return c - '0';
    } else if (c <= 'F') {
      return c - 'A' + 10;
    }
    __builtin_unreachable();
  };
  ++buffer;
  int sum = 0;
  while (*buffer && *buffer != '*') {
    sum ^= *buffer;
    ++buffer;
  }
  ++buffer;
  sum ^= to_hex(*(buffer + 0)) * 16 + to_hex(*(buffer + 1));
  return !sum;
}
}

int serial_port_open(const std::string &device) {
  auto descriptor = open(device.c_str(), O_RDWR | O_NDELAY | O_NOCTTY);
  if (descriptor < 0) {
    throw std::runtime_error(exception::strerrno(errno));
  }
  const auto n = fcntl(descriptor, F_GETFL, 0);
  fcntl(descriptor, F_SETFL, n & ~O_NDELAY);
  return descriptor;
}

void serial_port_close(int descriptor) { close(descriptor); }

void serial_port_configuration(int descriptor, baud_rate rate) {
  if (!isatty(descriptor)) {
    return;
  }

  termios tty;
  tcgetattr(descriptor, &tty);

  cfmakeraw(&tty);

  cfsetospeed(&tty, rate);
  cfsetispeed(&tty, rate);

  tty.c_cc[VMIN] = 1;
  tty.c_cc[VTIME] = 0;

  tcflush(descriptor, TCIFLUSH);
  if (tcsetattr(descriptor, TCSANOW, &tty) < 0) {
    throw std::runtime_error(exception::strerrno(errno));
  }
}

ssize_t serial_port_read(int descriptor, char *buffer) {
  static constexpr int minimum_sentence_size = 11;

  const char *orig_buffer = buffer;
  ssize_t bytes_read;

  char c;
  while ((bytes_read = read(descriptor, &c, 1)) > 0) {
    if (c == '$') {
      break;
    } else if (c == 0xA0) {
      if (read(descriptor, &c, 1) <= 0 || c != 0xA1) {
        continue;
      }
      char len[2];
      if (read(descriptor, len, sizeof(len)) != sizeof(len)) {
        continue;
      }

      *buffer++ = 0xA0;
      *buffer++ = 0xA1;
      *buffer++ = len[0];
      *buffer++ = len[1];

      const ssize_t payload_len = len[0] * 16 + len[1] + 3;
      ssize_t count = payload_len;
      while (count &&
             (bytes_read =
                  read(descriptor, buffer, static_cast<size_t>(count))) > 0) {
        count -= bytes_read;
        buffer += bytes_read;
      }
      return payload_len + 4;
    }
  }

  if (bytes_read <= 0) {
    return bytes_read;
  }

  *buffer++ = c;

  const ssize_t min_bytes_read =
      read(descriptor, buffer, minimum_sentence_size);
  if (min_bytes_read <= 0) {
    return min_bytes_read;
  }

  bytes_read += min_bytes_read;
  buffer += min_bytes_read;

  while (read(descriptor, &c, 1) > 0 && c != '\n') {
    *buffer++ = c;
    ++bytes_read;
  }

  *buffer++ = '\n';

  if (!check_nmea_sentence_checksum(orig_buffer)) {
    throw std::logic_error("failed checksum");
  }

  return bytes_read + 1;
}

ssize_t serial_port_write(int descriptor, const char *buffer, size_t len) {
  tcflush(descriptor, TCIOFLUSH);
  const auto bytes_written = write(descriptor, buffer, len);
  tcdrain(descriptor);
  if (bytes_written < 0) {
    throw std::runtime_error(exception::strerrno(errno));
  }
  return bytes_written;
}
}

inline namespace literals {
speed_t operator"" _bps(const char *baud_rate) {
  if (!std::strcmp(baud_rate, "4800")) {
    return B4800;
  } else if (!std::strcmp(baud_rate, "9600")) {
    return B9600;
  } else if (!std::strcmp(baud_rate, "19200")) {
    return B19200;
  } else if (!std::strcmp(baud_rate, "38400")) {
    return B38400;
  } else if (!std::strcmp(baud_rate, "57600")) {
    return B57600;
  } else if (!std::strcmp(baud_rate, "115200")) {
    return B115200;
  }
  __builtin_unreachable();
}
}

std::string mt33xx_stream_traits::_1hz() noexcept {
  return detail::PMTK_SET_NMEA_UPDATERATE_1HZ;
}

std::string mt33xx_stream_traits::_5hz() noexcept {
  return detail::PMTK_SET_NMEA_UPDATERATE_5HZ;
}

std::string mt33xx_stream_traits::_10hz() noexcept {
  return detail::PMTK_SET_NMEA_UPDATERATE_10HZ;
}

std::string mt33xx_stream_traits::nmea_default() noexcept {
  return detail::PMTK_SET_NMEA_OUTPUT_DEFAULT;
}

std::string mt33xx_stream_traits::off() noexcept {
  return detail::PMTK_SET_NMEA_OUTPUT_OFF;
}

std::string mt33xx_stream_traits::release() noexcept {
  return detail::PMTK_Q_RELEASE;
}

std::string skytraq_stream_traits::_1hz() noexcept {
  return detail::SKYTRAQ_SET_UPDATERATE_1HZ;
}

std::string skytraq_stream_traits::_2hz() noexcept {
  return detail::SKYTRAQ_SET_UPDATERATE_2HZ;
}

std::string skytraq_stream_traits::_4hz() noexcept {
  return detail::SKYTRAQ_SET_UPDATERATE_4HZ;
}

std::string skytraq_stream_traits::_5hz() noexcept {
  return detail::SKYTRAQ_SET_UPDATERATE_5HZ;
}

std::string skytraq_stream_traits::_8hz() noexcept {
  return detail::SKYTRAQ_SET_UPDATERATE_8HZ;
}

std::string skytraq_stream_traits::_10hz() noexcept {
  return detail::SKYTRAQ_SET_UPDATERATE_10HZ;
}

std::string skytraq_stream_traits::baud_rate_4800() noexcept {
  return detail::SKYTRAQ_SET_BAUD_RATE_4800;
}

std::string skytraq_stream_traits::baud_rate_9600() noexcept {
  return detail::SKYTRAQ_SET_BAUD_RATE_9600;
}

std::string skytraq_stream_traits::baud_rate_19200() noexcept {
  return detail::SKYTRAQ_SET_BAUD_RATE_19200;
}

std::string skytraq_stream_traits::baud_rate_38400() noexcept {
  return detail::SKYTRAQ_SET_BAUD_RATE_38400;
}

std::string skytraq_stream_traits::baud_rate_57600() noexcept {
  return detail::SKYTRAQ_SET_BAUD_RATE_57600;
}

std::string skytraq_stream_traits::baud_rate_115200() noexcept {
  return detail::SKYTRAQ_SET_BAUD_RATE_115200;
}

std::string skytraq_stream_traits::skytraq_binary() noexcept {
  return detail::SKYTRAQ_SET_OUTPUT_BINARY;
}

std::string skytraq_stream_traits::skytraq_nmea() noexcept {
  return detail::SKYTRAQ_SET_OUTPUT_NMEA;
}

std::string skytraq_stream_traits::nmea_default() noexcept {
  return detail::SKYTRAQ_SET_NMEA_OUTPUT_ALL;
}

std::string skytraq_stream_traits::off() noexcept {
  return detail::SKYTRAQ_SET_OUTPUT_OFF;
}

std::string skytraq_stream_traits::release() noexcept {
  return detail::SKYTRAQ_Q_RELEASE;
}

#ifdef __clang__
#pragma clang diagnostic push
#pragma clang diagnostic ignored "-Wexit-time-destructors"
#pragma clang diagnostic ignored "-Wglobal-constructors"
#endif
const std::string bdgsa_traits::value = "$BDGSA";
const std::string bdgsv_traits::value = "$BDGSV";
const std::string glgsa_traits::value = "$GLGSA";
const std::string glgsv_traits::value = "$GLGSV";
const std::string gngga_traits::value = "$GNGGA";
const std::string gngll_traits::value = "$GNGLL";
const std::string gngsa_traits::value = "$GNGSA";
const std::string gnrmc_traits::value = "$GNRMC";
const std::string gnvtg_traits::value = "$GNVTG";
const std::string gnzda_traits::value = "$GNZDA";
const std::string gpgga_traits::value = "$GPGGA";
const std::string gpgll_traits::value = "$GPGLL";
const std::string gpgsa_traits::value = "$GPGSA";
const std::string gpgsv_traits::value = "$GPGSV";
const std::string gprmc_traits::value = "$GPRMC";
const std::string gpvtg_traits::value = "$GPVTG";
const std::string gpzda_traits::value = "$GPZDA";
#ifdef __clang__
#pragma clang diagnostic pop
#endif
}
}
