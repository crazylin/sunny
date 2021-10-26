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

#ifndef BRANCH_IMAGE__BRANCH_IMAGE_HPP_
#define BRANCH_IMAGE__BRANCH_IMAGE_HPP_

#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"

namespace branch_image
{

class BranchImage : public rclcpp::Node
{
public:
  explicit BranchImage(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  virtual ~BranchImage();

private:
  void _InitializeParameters();
  void _UpdateParameters();

private:
  const char * _pubNameL = "~/image_l";
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr _pubL;

  const char * _pubNameR = "~/image_r";
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr _pubR;

  const char * _subName = "~/image";
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr _sub;
};

}  // namespace branch_image

#endif  // BRANCH_IMAGE__BRANCH_IMAGE_HPP_
