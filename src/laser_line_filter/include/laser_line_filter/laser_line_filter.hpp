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

#ifndef LASER_LINE_FILTER__LASER_LINE_FILTER_HPP_
#define LASER_LINE_FILTER__LASER_LINE_FILTER_HPP_

#include <memory>
#include <utility>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"

namespace laser_line_filter
{

class LaserLineFilter : public rclcpp::Node
{
public:
  explicit LaserLineFilter(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  virtual ~LaserLineFilter();

  void Publish(sensor_msgs::msg::PointCloud2::UniquePtr & ptr)
  {
    _pub->publish(std::move(ptr));
  }

private:
  const char * _pubName = "~/line_filtered";
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr _pub;

  class _Impl;
  std::unique_ptr<_Impl> _impl;

  const char * _subName = "~/line";
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr _sub;
};

}  // namespace laser_line_filter

#endif  // LASER_LINE_FILTER__LASER_LINE_FILTER_HPP_
