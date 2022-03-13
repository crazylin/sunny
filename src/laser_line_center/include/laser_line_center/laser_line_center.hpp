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

#ifndef LASER_LINE_CENTER__LASER_LINE_CENTER_HPP_
#define LASER_LINE_CENTER__LASER_LINE_CENTER_HPP_

#include <memory>
#include <utility>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/header.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "shared_interfaces/msg/line_center.hpp"

namespace laser_line_center
{

class LaserLineCenter : public rclcpp::Node
{
public:
  explicit LaserLineCenter(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  virtual ~LaserLineCenter();

  void Publish(shared_interfaces::msg::LineCenter::UniquePtr & msg)
  {
    _pub->publish(std::move(msg));
  }

  void Publish(const std_msgs::msg::Header & header)
  {
    _pubHeader->publish(header);
  }

private:
  const char * _pubName = "~/line";
  rclcpp::Publisher<shared_interfaces::msg::LineCenter>::SharedPtr _pub;

  const char * _pubHeaderName = "~/header";
  rclcpp::Publisher<std_msgs::msg::Header>::SharedPtr _pubHeader;

  class _Impl;
  std::unique_ptr<_Impl> _impl;

  const char * _subName = "~/image";
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr _sub;
};

}  // namespace laser_line_center

#endif  // LASER_LINE_CENTER__LASER_LINE_CENTER_HPP_
