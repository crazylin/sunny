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

#ifndef CAMERA_TIS__CAMERA_TIS_HPP_
#define CAMERA_TIS__CAMERA_TIS_HPP_

#include <utility>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"

namespace camera_tis
{

/**
 * @brief The imaging souce camera library (tiscamera), warpped in ROS2.
 *
 */
class CameraTis : public rclcpp::Node
{
public:
  /**
   * @brief Construct a new Camera Tis object.
   *
   * @param options Encapsulation of options for node initialization.
   */
  explicit CameraTis(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

  /**
   * @brief Destroy the Camera Tis object.
   *
   */
  virtual ~CameraTis();

  /**
   * @brief Publish an image msg via unique_ptr so intra process communication my be enabled if possible.
   *
   * @param ptr Reference to unique_ptr to be moved.
   */
  void publish(sensor_msgs::msg::Image::UniquePtr & ptr)
  {
    _pub->publish(std::move(ptr));
  }

private:
  /**
   * @brief Publisher name.
   *
   */
  const char * _pub_name = "~/image";

  /**
   * @brief Shared pointer to publisher.
   *
   */
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr _pub;

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
};

}  // namespace camera_tis

#endif  // CAMERA_TIS__CAMERA_TIS_HPP_
