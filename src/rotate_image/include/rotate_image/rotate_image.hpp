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

#ifndef ROTATE_IMAGE__ROTATE_IMAGE_HPP_
#define ROTATE_IMAGE__ROTATE_IMAGE_HPP_

#include <deque>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"

namespace rotate_image
{

using sensor_msgs::msg::Image;

/**
 * @brief Rotate images implemented via opencv's rotate.
 *
 */
class RotateImage : public rclcpp::Node
{
public:
  /**
   * @brief Construct a new Rotate Image object.
   *
   * Initialize publisher.
   * Declare parameters before usage.
   * Create a thread for each worker.
   * Create a thread for manager.
   * Initialize subscription.
   * Print success if all done.
   * @param options Encapsulation of options for node initialization.
   */
  explicit RotateImage(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

  /**
   * @brief Destroy the Rotate Image object.
   *
   * Release subscription.
   * Wake up all workers.
   * Wake up the manager.
   * Synchronize with all threads, wait for its return.
   * Release publisher.
   * Print success if all done.
   * Throw no exception.
   */
  virtual ~RotateImage();

private:
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
  void _push_back_future(std::future<Image::UniquePtr> f);

private:
  /**
   * @brief Publisher name.
   *
   */
  const char * _pub_name = "~/image_rotated";

  /**
   * @brief Shared pointer to publisher.
   *
   */
  rclcpp::Publisher<Image>::SharedPtr _pub;

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
   * @brief Rotation mode: 0: 90, 1: 180, 2: 270
   *
   */
  int _mode;

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
  std::deque<std::future<Image::UniquePtr>> _futures;

  /**
   * @brief Threads for workers and the manager.
   *
   */
  std::vector<std::thread> _threads;
};

}  // namespace rotate_image

#endif  // ROTATE_IMAGE__ROTATE_IMAGE_HPP_
