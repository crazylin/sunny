// Copyright 2019 Zhushi Tech, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "gpio_raspberry/gpio_raspberry.hpp"

#include <gpiod.h>

#include <vector>

namespace gpio_raspberry
{

using rcl_interfaces::msg::ParameterDescriptor;
using rcl_interfaces::msg::SetParametersResult;

GpioRaspberry::GpioRaspberry(const rclcpp::NodeOptions & options)
: Node("gpio_raspberry_node", options),
  _chip(gpiod_chip_open_by_name("gpiochip0"), gpiod_chip_close),
  _line_26(gpiod_chip_get_line(_chip.get(), 26), gpiod_line_release),
  _line_6(gpiod_chip_get_line(_chip.get(), 6), gpiod_line_release)
{
  // To enforce start with laser off
  this->declare_parameter("laser", false, ParameterDescriptor(), true);

  gpiod_line_request_output(_line_26.get(), "ros", 0);
  gpiod_line_request_output(_line_6.get(), "ros", 0);
  gpiod_line_set_value(_line_6.get(), 1);

  _handle = this->add_on_set_parameters_callback(
    [this](const std::vector<rclcpp::Parameter> & parameters) {
      SetParametersResult result;
      result.successful = true;
      for (const auto & p : parameters) {
        if (p.get_name() == "laser") {
          auto ret = this->_laser(p.as_bool());
          if (ret) {
            result.successful = false;
            result.reason = "Failed to set laser";
            return result;
          }
        }
      }
      return result;
    });

  RCLCPP_INFO(this->get_logger(), "Initialized successfully");
}

GpioRaspberry::~GpioRaspberry()
{
  try {
    RCLCPP_INFO(this->get_logger(), "Destroyed successfully");
  } catch (const std::exception & e) {
    RCLCPP_FATAL(this->get_logger(), "Exception in destructor: %s", e.what());
  } catch (...) {
    RCLCPP_FATAL(this->get_logger(), "Exception in destructor: unknown");
  }
}

int GpioRaspberry::_laser(bool f)
{
  if (f) {
    return gpiod_line_set_value(_line_26.get(), 1);
  } else {
    return gpiod_line_set_value(_line_26.get(), 0);
  }
}

}  // namespace gpio_raspberry

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(gpio_raspberry::GpioRaspberry)
