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

class CameraTis : public rclcpp::Node
{
public:
  explicit CameraTis(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  virtual ~CameraTis();

  void publish(sensor_msgs::msg::Image::UniquePtr & ptr)
  {
    _pubImage->publish(std::move(ptr));
  }

private:
  const char * _pubImageName = "~/image";
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr _pubImage;

  class _Impl;
  std::unique_ptr<_Impl> _impl;
};

}  // namespace camera_tis

#endif  // CAMERA_TIS__CAMERA_TIS_HPP_
