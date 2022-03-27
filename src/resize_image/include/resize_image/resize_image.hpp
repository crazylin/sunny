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

#ifndef RESIZE_IMAGE__RESIZE_IMAGE_HPP_
#define RESIZE_IMAGE__RESIZE_IMAGE_HPP_

#include <memory>
#include <utility>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"

namespace resize_image
{

class ResizeImage : public rclcpp::Node
{
public:
  explicit ResizeImage(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  virtual ~ResizeImage();

  void Publish(sensor_msgs::msg::Image::UniquePtr & ptr)
  {
    _pubImage->publish(std::move(ptr));
  }

private:
  const char * _pubImageName = "~/image_resized";
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr _pubImage;

  class _Impl;
  std::unique_ptr<_Impl> _impl;

  const char * _subName = "~/image";
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr _sub;
};

}  // namespace resize_image

#endif  // RESIZE_IMAGE__RESIZE_IMAGE_HPP_
