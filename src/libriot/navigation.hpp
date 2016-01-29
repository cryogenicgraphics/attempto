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
#include <cassert>
#include <cstring>
#include <iostream>
#include <memory>
#include <streambuf>
#include <string>

namespace riot {
namespace navigation {

using baud_rate = unsigned int;

namespace detail {
int serial_port_open(const std::string &device);
void serial_port_close(int descriptor);
void serial_port_configuration(int descriptor, baud_rate rate);
ssize_t serial_port_read(int descriptor, char *buffer);
ssize_t serial_port_write(int descriptor, const char *buffer, size_t len);
}

inline namespace literals { baud_rate operator"" _bps(const char *rate); }

struct basic_stream_traits {
  constexpr static std::size_t uncompressed_linesize() noexcept { return 128; }
  constexpr static bool nmea() noexcept { return true; }
  constexpr static bool skytraq() noexcept { return false; }
};

struct mt33xx_stream_traits final : public basic_stream_traits {
  static std::string _1hz() noexcept;
  static std::string _5hz() noexcept;
  static std::string _10hz() noexcept;
  static std::string nmea_default() noexcept;
  static std::string off() noexcept;
  static std::string release() noexcept;
};

struct skytraq_stream_traits final : public basic_stream_traits {
  constexpr static bool skytraq() noexcept { return true; }
  static std::string _1hz() noexcept;
  static std::string _2hz() noexcept;
  static std::string _4hz() noexcept;
  static std::string _5hz() noexcept;
  static std::string _8hz() noexcept;
  static std::string _10hz() noexcept;
  static std::string baud_rate_4800() noexcept;
  static std::string baud_rate_9600() noexcept;
  static std::string baud_rate_19200() noexcept;
  static std::string baud_rate_38400() noexcept;
  static std::string baud_rate_57600() noexcept;
  static std::string baud_rate_115200() noexcept;
  static std::string skytraq_binary() noexcept;
  static std::string skytraq_nmea() noexcept;
  static std::string nmea_default() noexcept;
  static std::string off() noexcept;
  static std::string release() noexcept;
};

template <class stream_traits, class charT,
          class char_traits = std::char_traits<charT>>
class basic_gnss_streambuf final
    : public std::basic_streambuf<charT, char_traits> {
  using strmbuf = std::basic_streambuf<charT, char_traits>;

  constexpr auto linesize() const noexcept {
    return stream_traits::uncompressed_linesize();
  }

public:
  explicit basic_gnss_streambuf(const std::string &device, baud_rate rate)
      : descriptor_(detail::serial_port_open(device)) {
    detail::serial_port_configuration(descriptor_, rate);

    strmbuf::setg(&buffer_[linesize()], &buffer_[linesize()],
                  &buffer_[linesize()]);
  }

  basic_gnss_streambuf(const basic_gnss_streambuf &) = delete;
  basic_gnss_streambuf &operator=(const basic_gnss_streambuf &) = delete;

  basic_gnss_streambuf(basic_gnss_streambuf &&) = default;
  basic_gnss_streambuf &operator=(basic_gnss_streambuf &&) = default;

  virtual ~basic_gnss_streambuf() override {
    detail::serial_port_close(descriptor_);
  }

  void set(const std::string &cmd) const {
    detail::serial_port_write(descriptor_, cmd.c_str(), cmd.size());
  }

protected:
  virtual typename char_traits::int_type underflow() override {
    if (strmbuf::gptr() < strmbuf::egptr()) {
      return char_traits::to_int_type(*strmbuf::gptr());
    }

    const auto num_putback = std::min(
        static_cast<size_t>(strmbuf::gptr() - strmbuf::eback()), linesize());

    std::memmove(&buffer_[0] + (linesize() - num_putback),
                 strmbuf::gptr() - num_putback, num_putback);

    const auto read =
        detail::serial_port_read(descriptor_, &buffer_[linesize()]);
    if (read <= 0) {
      return char_traits::eof();
    }

    strmbuf::setg(&buffer_[0] + (linesize() - num_putback),
                  &buffer_[0] + linesize(), &buffer_[0] + linesize() + read);

    return char_traits::to_int_type(*strmbuf::gptr());
  }

  virtual std::streamsize xsgetn(charT *s, std::streamsize) override {
    // assert count >= linesize
    strmbuf::setg(&buffer_[linesize()], &buffer_[linesize()],
                  &buffer_[linesize()]);

    return detail::serial_port_read(descriptor_, s);
  }

private:
  // allow linesize putback characters and linesize read (2 lines)
  std::array<charT, 2 * stream_traits::uncompressed_linesize()> buffer_
      __attribute__((aligned(16)));
  int descriptor_;
};

using gnss_mt33xx_streambuf = basic_gnss_streambuf<mt33xx_stream_traits, char>;
using gnss_skytraq_streambuf =
    basic_gnss_streambuf<skytraq_stream_traits, char>;

#if __GNUG__ > 4
static_assert(std::is_move_constructible<gnss_mt33xx_streambuf>::value,
              "must be move constructible");
static_assert(std::is_move_constructible<gnss_skytraq_streambuf>::value,
              "must be move constructible");

static_assert(std::is_move_assignable<gnss_mt33xx_streambuf>::value,
              "must be move assignable");
static_assert(std::is_move_assignable<gnss_skytraq_streambuf>::value,
              "must be move assignable");
#endif

template <class stream_traits, class charT,
          class char_traits = std::char_traits<charT>>
class basic_gnss_istream final : public std::basic_istream<charT, char_traits> {
  using istream = std::basic_istream<charT, char_traits>;

public:
  explicit basic_gnss_istream(const std::string &device,
                              baud_rate rate = 115200_bps) try
      : istream(0),
        streambuf_(device, rate) {
    istream::rdbuf(&streambuf_);
  } catch (const std::exception &) {
    istream::setstate(std::ios::badbit);
  }

  basic_gnss_istream(basic_gnss_istream &&rhs) : istream() {
    istream::move(rhs);
    streambuf_.move(rhs.streambuf_);
  }
  basic_gnss_istream &operator=(basic_gnss_istream &&rhs) {
    swap(rhs);
    return *this;
  }

  void swap(basic_gnss_istream &rhs) {
    istream::swap(rhs);
    std::swap(streambuf_, rhs.streambuf_);
  }

  using istream::operator>>;

  basic_gnss_istream &
  operator>>(basic_gnss_istream &(*pf)(basic_gnss_istream &)) {
    return pf(*this);
  }

  void set(const std::string &cmd) const { streambuf_.set(cmd); }

private:
  basic_gnss_streambuf<stream_traits, charT, char_traits> streambuf_;
};

using gnss_mt33xx_istream = basic_gnss_istream<mt33xx_stream_traits, char>;
using gnss_skytraq_istream = basic_gnss_istream<skytraq_stream_traits, char>;

#if __GNUG__ > 4
static_assert(std::is_move_constructible<gnss_mt33xx_istream>::value,
              "must be move constructible");
static_assert(std::is_move_constructible<gnss_skytraq_istream>::value,
              "must be move constructible");

static_assert(std::is_move_assignable<gnss_mt33xx_istream>::value,
              "must be move assignable");
static_assert(std::is_move_assignable<gnss_skytraq_istream>::value,
              "must be move assignable");
#endif
}
}

#include "manipulator.hpp"
#include "detail/struct.hpp"
