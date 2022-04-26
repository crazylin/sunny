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

#ifndef CAMERA_TIS__CAMERA_TIS_HPP_
#define CAMERA_TIS__CAMERA_TIS_HPP_

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"

struct _GstElement;

namespace camera_tis
{

using sensor_msgs::msg::Image;

/**
 * @brief The imaging souce camera library (tiscamera), warpped in ROS2.
 *
 */
class CameraTis : public rclcpp::Node
{
public:
  /**
   * @brief Construct a new Camera Tis object.
   *
   * Initialize publisher.
   * Create an inner implementation.
   * Print success if all done.
   * @param options Encapsulation of options for node initialization.
   */
  explicit CameraTis(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

  /**
   * @brief Destroy the Camera Tis object.
   *
   * Release inner implementation.
   * Release publisher.
   * Print success if all done.
   * Throw no exception.
   */
  virtual ~CameraTis();

private:
  /**
   * @brief Declare parameters with defaults before usage.
   *
   */
  void _declare_parameters();

  /**
   * @brief Construct a new impl object.
   *
   * Declare parameters before usage.
   * Initialize gst environment.
   * Create pipeline.
   * Set default properties.
   * Create spin thread.
   * Initialize ROS parameter callback.
   */
  void _initialize_camera();

  /**
   * @brief Spin infinitely to receive image data from camera.
   *
   */
  void _spin();

  /**
   * @brief Get the camera's state: capturing or not.
   *
   * @return true Capturing.
   * @return false Not capturing.
   */
  bool _power();

  /**
   * @brief Enable capture.
   *
   * @return int 0 if success.
   */
  int _power_on();

  /**
   * @brief Disable capture.
   *
   * @return int 0 if success.
   */
  int _power_off();

  /**
   * @brief Set the camera's state: capturing or not.
   *
   * @param p true to enable capture.
   * @return int 0 if success.
   */
  int _set_power(bool p);

  /**
   * @brief Set the exposure object.
   *
   * @param e int in microseconds.
   * @return int 0 if success.
   */
  int _set_exposure(int e);

private:
  /**
   * @brief Publisher name.
   *
   */
  const char * _pub_name = "~/image";

  /**
   * @brief Shared pointer to publisher.
   *
   */
  rclcpp::Publisher<Image>::SharedPtr _pub;

  /**
   * @brief The handle to the camera pipeline
   *
   */
  _GstElement * _pipeline;

  /**
   * @brief Thread spin infinitely to capture images.
   *
   */
  std::thread _thread;

  /**
   * @brief Callback handle for parameters.
   *
   */
  OnSetParametersCallbackHandle::SharedPtr _handle;
};

}  // namespace camera_tis

#endif  // CAMERA_TIS__CAMERA_TIS_HPP_
