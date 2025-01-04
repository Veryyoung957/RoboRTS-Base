/****************************************************************************
 *  Copyright (C) 2021 RoboMaster.
 *
 *  This program is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of 
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program. If not, see <http://www.gnu.org/licenses/>.
 ***************************************************************************/

#ifndef ROBORTS_BASE_CONFIG_H
#define ROBORTS_BASE_CONFIG_H

#include <rclcpp/rclcpp.hpp>

namespace roborts_base{

struct Config : public rclcpp::Node {
  Config() : Node("config"){
    this->declare_parameter("serial_port", "/dev/ttyACM0");
    this->declare_parameter<std::vector<std::string>>("load_module", 
                                        {"chassis",
                                         "gimbal",
                                         "referee_system"});
    load_module = {"chassis",
                    "gimbal",
                    "referee_system"};
    serial_port = this->get_parameter("serial_port").as_string();
  }
  // void GetParam() {
  //   this->declare_parameter<std::string>("serial_port", "/dev/ttyACM0");
  //   this->declare_parameter<std::vector<std::string>>("load_module", 
  //                                       {"chassis",
  //                                        "gimbal",
  //                                        "referee_system"});
  // }
  std::string serial_port;
  std::vector<std::string> load_module;
};

}
#endif //ROBORTS_BASE_CONFIG_H