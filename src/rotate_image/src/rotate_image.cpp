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

#include "rotate_image/rotate_image.hpp"

#include <utility>

#include "opencv2/opencv.hpp"

namespace rotate_image
{

/**
 * @brief Extract extra 'worker' parameter from ROS node options.
 *
 * @param options Encapsulation of options for node initialization.
 * @return int Number of workers.
 */
int workers(const rclcpp::NodeOptions & options)
{
  for (const auto & p : options.parameter_overrides()) {
    if (p.get_name() == "workers") {
      return p.as_int();
    }
  }
  return 1;
}

RotateImage::RotateImage(const rclcpp::NodeOptions & options)
: Node("rotate_image_node", options)
{
  _pub = this->create_publisher<Image>(_pub_name, rclcpp::SensorDataQoS());

  this->declare_parameter("mode", 0);
  _mode = this->get_parameter("mode").as_int();
  _workers = workers(options);
  for (int i = 0; i < _workers; ++i) {
    _threads.push_back(std::thread(&RotateImage::_worker, this));
  }
  _threads.push_back(std::thread(&RotateImage::_manager, this));

  _sub = this->create_subscription<Image>(
    _sub_name,
    rclcpp::SensorDataQoS(),
    [this](Image::UniquePtr ptr)
    {
      _push_back_image(std::move(ptr));
    }
  );

  RCLCPP_INFO(this->get_logger(), "Ininitialized successfully");
}

RotateImage::~RotateImage()
{
  try {
    _sub.reset();
    _images_con.notify_all();
    _futures_con.notify_one();
    for (auto & t : _threads) {
      t.join();
    }
    _pub.reset();

    RCLCPP_INFO(this->get_logger(), "Destroyed successfully");
  } catch (const std::exception & e) {
    RCLCPP_ERROR(this->get_logger(), "Exception in destructor: %s", e.what());
  } catch (...) {
    RCLCPP_ERROR(this->get_logger(), "Exception in destructor: unknown");
  }
}

void RotateImage::_worker()
{
  Image::_data_type buf;
  while (rclcpp::ok()) {
    std::unique_lock<std::mutex> lk(_images_mut);
    if (_images.empty() == false) {
      auto ptr = std::move(_images.front());
      _images.pop_front();
      std::promise<Image::UniquePtr> prom;
      _push_back_future(prom.get_future());
      lk.unlock();
      if (ptr->header.frame_id == "-1" || ptr->data.empty()) {
        prom.set_value(std::move(ptr));
      } else {
        buf.resize(ptr->data.size());
        cv::Mat src(ptr->height, ptr->width, CV_8UC1, ptr->data.data());
        cv::Mat dst(ptr->width, ptr->height, CV_8UC1, buf.data());
        // cv::rotate(src, dst, cv::ROTATE_90_CLOCKWISE);
        cv::rotate(src, dst, _mode);
        std::swap(ptr->data, buf);
        std::swap(ptr->width, ptr->height);
        ptr->step = ptr->width;
        prom.set_value(std::move(ptr));
      }
    } else {
      _images_con.wait(lk);
    }
  }
}

void RotateImage::_manager()
{
  while (rclcpp::ok()) {
    std::unique_lock<std::mutex> lk(_futures_mut);
    if (_futures.empty() == false) {
      auto f = std::move(_futures.front());
      _futures.pop_front();
      lk.unlock();
      auto ptr = f.get();
      _pub->publish(std::move(ptr));
    } else {
      _futures_con.wait(lk);
    }
  }
}

void RotateImage::_push_back_image(Image::UniquePtr ptr)
{
  std::unique_lock<std::mutex> lk(_images_mut);
  _images.emplace_back(std::move(ptr));
  auto s = static_cast<int>(_images.size());
  if (s > _workers + 1) {
    _images.pop_front();
  }
  lk.unlock();
  _images_con.notify_all();
}

void RotateImage::_push_back_future(std::future<Image::UniquePtr> f)
{
  std::unique_lock<std::mutex> lk(_futures_mut);
  _futures.emplace_back(std::move(f));
  lk.unlock();
  _futures_con.notify_one();
}

}  // namespace rotate_image

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(rotate_image::RotateImage)
