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

namespace line_center_reconstruction
{

/**
 * @brief Homography transformation between two plane: image plane and laser plane.
 *
 */
class LineCenterReconstruction : public rclcpp::Node
{
public:
  /**
   * @brief Construct a new Line Center Reconstruction object.
   *
   * @param options Encapsulation of options for node initialization.
   */
  explicit LineCenterReconstruction(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

  /**
   * @brief Destroy the Line Center Reconstruction object.
   *
   */
  virtual ~LineCenterReconstruction();

  /**
   * @brief Publish an point cloud msg via unique_ptr so intra process communication my be enabled if possible.
   *
   * @param ptr Reference to unique_ptr to be moved.
   */
  void publish(sensor_msgs::msg::PointCloud2::UniquePtr & ptr)
  {
    _pub->publish(std::move(ptr));
  }

private:
  /**
   * @brief Publisher name.
   *
   */
  const char * _pub_name = "~/pnts";

  /**
   * @brief Shared pointer to publisher.
   *
   */
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr _pub;

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
   * @brief Subscription name.
   *
   */
  const char * _sub_name = "~/line";

  /**
   * @brief Shared pointer to subscription.
   *
   */
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr _sub;
};

}  // namespace line_center_reconstruction

#endif  // LINE_CENTER_RECONSTRUCTION__LINE_CENTER_RECONSTRUCTION_HPP_
