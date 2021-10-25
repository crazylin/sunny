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

#ifndef LINE_CENTER_RECONSTRUCTION__LINE_CENTER_RECONSTRUCTION_HPP_
#define LINE_CENTER_RECONSTRUCTION__LINE_CENTER_RECONSTRUCTION_HPP_

#include <memory>
#include <utility>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "shared_interfaces/msg/line_center.hpp"

namespace line_center_reconstruction
{

class LineCenterReconstruction : public rclcpp::Node
{
public:
  explicit LineCenterReconstruction(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  virtual ~LineCenterReconstruction();

  void Publish(sensor_msgs::msg::PointCloud2::UniquePtr & msg)
  {
    _pub->publish(std::move(msg));
  }

private:
  const char * _pubName = "~/pnts";
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr _pub;

  class _Impl;
  std::unique_ptr<_Impl> _impl;

  const char * _subName = "~/line";
  rclcpp::Subscription<shared_interfaces::msg::LineCenter>::SharedPtr _sub;
};

}  // namespace line_center_reconstruction

#endif  // LINE_CENTER_RECONSTRUCTION__LINE_CENTER_RECONSTRUCTION_HPP_
