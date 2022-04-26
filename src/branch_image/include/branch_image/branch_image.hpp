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

/**
 * @brief Branch input images based on image's frame ID.
 *
 */
class BranchImage : public rclcpp::Node
{
public:
  /**
   * @brief Construct a new Branch Image object.
   *
   * @param options Encapsulation of options for node initialization.
   */
  explicit BranchImage(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

  /**
   * @brief Destroy the Branch Image object.
   *
   */
  virtual ~BranchImage();

private:
  /**
   * @brief Initialize ROS parameters, declare before get.
   *
   */
  void _InitializeParameters();

  /**
   * @brief Update ROS parameters.
   *
   */
  void _UpdateParameters();

private:
  /**
   * @brief Publisher name for image with even frame ID.
   *
   */
  const char * _pubNameL = "~/image_l";

  /**
   * @brief Shared pointer to publisher for image with even frame ID.
   *
   */
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr _pubL;

  /**
   * @brief Publisher name for image with odd frame ID.
   *
   */
  const char * _pubNameR = "~/image_r";

  /**
   * @brief Shared pointer to publisher for image with odd frame ID.
   *
   */
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr _pubR;

  /**
   * @brief Subscription name.
   *
   */
  const char * _subName = "~/image";

  /**
   * @brief Shared pointer to subscription.
   *
   */
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr _sub;
};

}  // namespace branch_image

#endif  // BRANCH_IMAGE__BRANCH_IMAGE_HPP_
