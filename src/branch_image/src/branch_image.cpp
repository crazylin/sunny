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

#include "branch_image/branch_image.hpp"

#include <memory>
#include <utility>

namespace branch_image
{

using sensor_msgs::msg::Image;

/**
 * @brief Construct a new Branch Image object.
 *
 * Initialize parameters then get parameters.
 * Initialize two publisher and then one subscription.
 * Print success if all done.
 * @param options Encapsulation of options for node initialization.
 */
BranchImage::BranchImage(const rclcpp::NodeOptions & options)
: Node("branch_image_node", options)
{
  _InitializeParameters();

  _UpdateParameters();

  _pubL = this->create_publisher<Image>(_pubNameL, rclcpp::SensorDataQoS());

  _pubR = this->create_publisher<Image>(_pubNameR, rclcpp::SensorDataQoS());

  _sub = this->create_subscription<Image>(
    _subName,
    rclcpp::SensorDataQoS(),
    [this](Image::UniquePtr ptr)
    {
      if (std::stoi(ptr->header.frame_id) % 2) {
        _pubL->publish(std::move(ptr));
      } else {
        _pubR->publish(std::move(ptr));
      }
    }
  );

  RCLCPP_INFO(this->get_logger(), "Ininitialized successfully");
}

/**
 * @brief Destroy the Branch Image object.
 *
 * Release subscription.
 * Release publishers.
 * Print success if all done.
 */
BranchImage::~BranchImage()
{
  _sub.reset();
  _pubL.reset();
  _pubR.reset();

  RCLCPP_INFO(this->get_logger(), "Destroyed successfully");
}

/**
 * @brief Initialize parameters before usage.
 *
 */
void BranchImage::_InitializeParameters()
{
  // this->declare_parameter("");
}

/**
 * @brief Update parameters.
 *
 */
void BranchImage::_UpdateParameters()
{
  // this->get_parameter("", );
}

}  // namespace branch_image

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(branch_image::BranchImage)
