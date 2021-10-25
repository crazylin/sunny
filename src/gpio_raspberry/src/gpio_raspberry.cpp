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
#include <fstream>
#include <memory>
#include <string>

namespace gpio_raspberry
{

class GpioRaspberry::_Impl
{
public:
  _Impl()
  : _chip(gpiod_chip_open_by_name("gpiochip0"), gpiod_chip_close),
    _line(gpiod_chip_get_line(_chip.get(), 26), gpiod_line_release)
  {
    gpiod_line_request_output(_line.get(), "ros", 0);
  }

  ~_Impl()
  {
  }

  int High()
  {
    return gpiod_line_set_value(_line.get(), 1);
  }

  int Low()
  {
    return gpiod_line_set_value(_line.get(), 0);
  }

private:
  std::unique_ptr<gpiod_chip, void (*)(gpiod_chip *)> _chip;
  std::unique_ptr<gpiod_line, void (*)(gpiod_line *)> _line;
};

using std_srvs::srv::Trigger;

GpioRaspberry::GpioRaspberry(const rclcpp::NodeOptions & options)
: Node("gpio_raspberry_node", options), _impl(new _Impl)
{
  _InitializeParameters();

  _UpdateParameters();

  _srvHigh = this->create_service<Trigger>(
    _srvHighName,
    [this](const std::shared_ptr<Trigger::Request>, std::shared_ptr<Trigger::Response> response)
    {
      if (_impl->High()) {
        response->success = false;
        response->message = "Failed: IO set to high";
      } else {
        response->success = true;
        response->message = "Success: IO set to high";
      }
    }
  );

  _srvLow = this->create_service<Trigger>(
    _srvLowName,
    [this](const std::shared_ptr<Trigger::Request>, std::shared_ptr<Trigger::Response> response)
    {
      if (_impl->Low()) {
        response->success = false;
        response->message = "Failed: IO set to low";
      } else {
        response->success = true;
        response->message = "Success: IO set to low";
      }
    }
  );

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

void GpioRaspberry::_InitializeParameters()
{
  // this->declare_parameter("port", _port);
}

void GpioRaspberry::_UpdateParameters()
{
  // this->get_parameter("port", _port);
}

}  // namespace gpio_raspberry

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(gpio_raspberry::GpioRaspberry)
