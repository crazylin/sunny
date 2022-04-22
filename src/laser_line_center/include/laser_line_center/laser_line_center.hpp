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

#include <deque>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"

namespace laser_line_center
{

using sensor_msgs::msg::Image;
using sensor_msgs::msg::PointCloud2;

/**
 * @brief List of parameter names.
 *
 */
const std::vector<std::string> KEYS = {"ksize", "threshold", "width_min", "width_max"};

/**
 * @brief To zip related parameters together.
 *
 */
struct Params
{
  int ksize = 5;
  int threshold = 35;
  double width_min = 1.;
  double width_max = 30.;
};

/**
 * @brief High performance line center ridge extraction algorithm with sub-pixel accuracy.
 *
 */
class LaserLineCenter : public rclcpp::Node
{
public:
  /**
   * @brief Construct a new Laser Line Center object.
   *
   * Initialize publisher.
   * Create an inner implementation.
   * Initialize subscription.
   * Print success if all done.
   * @param options Encapsulation of options for node initialization.
   */
  explicit LaserLineCenter(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

  /**
   * @brief Destroy the Laser Line Center object.
   *
   * Release subscription.
   * Wake up all workers.
   * Wake up the manager.
   * Synchronize with all threads, wait for its return.
   * Release publisher.
   * Print success if all done.
   * Throw no exception.
   */
  virtual ~LaserLineCenter();

private:
  /**
   * @brief Declare parameters with defaults before usage.
   *
   */
  void _declare_parameters();

  /**
   * @brief Update parameters from ROS.
   *
   * @return Params Zipped parameters
   */
  Params _update_parameters();

  /**
   * @brief The worker works in seperate thread to process incoming date parallelly.
   *
   * Create a buffer.
   * Enter infinite loop.
   * Wait for incoming data.
   * Wake up to get a possible data, make a promise and notify the manager.
   * Continue to work on the data and return to sleep if no further data to process.
   */
  void _worker();

  /**
   * @brief The manager works in seperate thread to gather worker's results in order.
   *
   * Spin infinitely until rclcpp:ok() return false.
   * Whenever a future is ready, the manager wake up, get the result from the future and publish.
   */
  void _manager();

  /**
   * @brief Push a image and notity workers.
   *
   * @param ptr Reference to a unique pointer to image to be moved.
   */
  void _push_back_image(Image::UniquePtr ptr);

  /**
   * @brief Promise a future so its future can be sychronized and notify the manager.
   *
   * @param f A future to point cloud msg.
   */
  void _push_back_future(std::future<PointCloud2::UniquePtr> fut);

private:
  /**
   * @brief Publisher name.
   *
   */
  const char * _pub_name = "~/line";

  /**
   * @brief Shared pointer to publisher.
   *
   */
  rclcpp::Publisher<PointCloud2>::SharedPtr _pub;

  /**
   * @brief Subscription name.
   *
   */
  const char * _sub_name = "~/image";

  /**
   * @brief Shared pointer to subscription.
   *
   */
  rclcpp::Subscription<Image>::SharedPtr _sub;

  /**
   * @brief Number of co-workers.
   *
   */
  int _workers;

  /**
   * @brief Mutex to protect image queue.
   *
   */
  std::mutex _images_mut;

  /**
   * @brief Condition variable for image queue.
   *
   */
  std::condition_variable _images_con;

  /**
   * @brief Double end queue for images.
   *
   */
  std::deque<Image::UniquePtr> _images;

  /**
   * @brief Mutex to protect result queue.
   *
   */
  std::mutex _futures_mut;

  /**
   * @brief Condition variable for result queue.
   *
   */
  std::condition_variable _futures_con;

  /**
   * @brief Double end queue for results.
   *
   */
  std::deque<std::future<PointCloud2::UniquePtr>> _futures;

  /**
   * @brief Threads for workers and the manager.
   *
   */
  std::vector<std::thread> _threads;

  /**
   * @brief Callback handle for parameters.
   *
   */
  OnSetParametersCallbackHandle::SharedPtr _handle;
};

}  // namespace laser_line_center

#endif  // LASER_LINE_CENTER__LASER_LINE_CENTER_HPP_
