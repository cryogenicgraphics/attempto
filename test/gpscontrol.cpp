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

#include <cstdlib>
#include <iostream>
#include <memory>

#include <boost/program_options.hpp>

#include <riot/navigation.hpp>

namespace nav = riot::navigation;
namespace po = boost::program_options;

namespace {
static po::variables_map register_command_line_options(int argc, char *argv[]) {
  po::options_description desc("options");
  auto &&options = desc.add_options();

  options("command", po::value<std::string>(), "default|off|release");
  options("device", po::value<std::string>(),
          "/path/to/device (e.g. /dev/ttyO4");
  options("help", "help message");
  options("skytraq", "instantiate a skytraq device");

  po::variables_map vm;
  po::store(po::parse_command_line(argc, argv, desc), vm);
  po::notify(vm);

  if (vm.count("help")) {
    std::cout << desc << std::endl;
    std::exit(0);
  }
  return vm;
}

void run_mt33xx(const po::variables_map &vm) {
  nav::gnss_mt33xx_istream strm(vm["device"].as<std::string>());
  strm >> std::noskipws;

  if (vm.count("command")) {
    const auto &command = vm["command"].as<std::string>();
    if (command == "off") {
      strm >> nav::off;
    } else if (command == "release") {
      strm >> nav::release;
    } else if (command == "default") {
      strm >> nav::nmea_default;
    } else {
      std::cerr << "Unknown command. Ignoring..." << std::endl;
    }
  }

  std::copy(std::istream_iterator<char>(strm), std::istream_iterator<char>(),
            std::ostream_iterator<char>(std::cout));
}

void run_skytraq(const po::variables_map &vm) {
  nav::gnss_skytraq_istream strm(vm["device"].as<std::string>());
  strm >> std::noskipws;

  if (vm.count("command")) {
    const auto &command = vm["command"].as<std::string>();
    if (command == "binary") {
      strm >> nav::skytraq_binary;
    } else if (command == "default") {
      strm >> nav::nmea_default;
    } else if (command == "nmea") {
      strm >> nav::skytraq_nmea;
    } else if (command == "off") {
      strm >> nav::off;
    } else if (command == "release") {
      nav::skytraq::version version;
      strm >> nav::release >> version;
    } else {
      std::cerr << "Unknown command. Ignoring..." << std::endl;
    }
  }

  std::copy(std::istream_iterator<char>(strm), std::istream_iterator<char>(),
            std::ostream_iterator<char>(std::cout));
}
}

int main(int argc, char *argv[]) {
  try {
    const auto &vm = register_command_line_options(argc, argv);

    if (!vm.count("device")) {
      std::cerr << "Please specify a device. Aborting..." << std::endl;
      return 1;
    }

    if (vm.count("skytraq")) {
      run_skytraq(vm);
    } else {
      run_mt33xx(vm);
    }
  } catch (const std::exception &ex) {
    std::cerr << ex.what() << std::endl;
    return 1;
  }
}
