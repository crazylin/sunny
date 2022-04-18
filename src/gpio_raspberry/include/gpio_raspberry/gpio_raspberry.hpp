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

#ifndef GPIO_RASPBERRY__GPIO_RASPBERRY_HPP_
#define GPIO_RASPBERRY__GPIO_RASPBERRY_HPP_

#include <memory>

#include "rclcpp/rclcpp.hpp"

namespace gpio_raspberry
{

/**
 * @brief Control GPIO, and laser by using libgpiod.
 *
 */
class GpioRaspberry : public rclcpp::Node
{
public:
  /**
   * @brief Construct a new Gpio Raspberry object.
   *
   * @param options Encapsulation of options for node initialization.
   */
  explicit GpioRaspberry(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

  /**
   * @brief Destroy the Gpio Raspberry object.
   *
   */
  virtual ~GpioRaspberry();

private:
  /**
   * @brief Forward declaration for inner implementation.
   *
   */
  class _Impl;

  /**
   * @brief Unique pointer to inner implementation.
   *
   */
  std::unique_ptr<_Impl> _impl;

  /**
   * @brief ROS parameter callback handle.
   *
   */
  OnSetParametersCallbackHandle::SharedPtr _handle;
};

}  // namespace gpio_raspberry

#endif  // GPIO_RASPBERRY__GPIO_RASPBERRY_HPP_
