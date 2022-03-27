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

#include "resize_image/resize_image.hpp"

#include <deque>
#include <exception>
#include <future>
#include <map>
#include <memory>
#include <utility>
#include <vector>

#include "opencv2/opencv.hpp"

namespace resize_image
{

using sensor_msgs::msg::Image;

class ResizeImage::_Impl
{
public:
  explicit _Impl(RisizeImage * ptr)
  : _node(ptr)
  {
    _node->declare_parameter("workers", _workers);
    _node->get_parameter("workers", _workers);
    _deque_size = _workers + 1;
    for (int i = 0; i < _workers; ++i) {
      _threads.push_back(std::thread(&_Impl::_Worker, this));
    }
    _threads.push_back(std::thread(&_Impl::_Manager, this));
    RCLCPP_INFO(_node->get_logger(), "Employ %d workers successfully", _workers);
  }

  ~_Impl()
  {
    _images_con.notify_all();
    _futures_con.notify_one();
    for (auto & t : _threads) {
      t.join();
    }
  }

  void PushBackImage(Image::UniquePtr ptr)
  {
    std::unique_lock<std::mutex> lk(_images_mut);
    _images.emplace_back(std::move(ptr));
    auto s = static_cast<int>(_images.size());
    if (s > _deque_size) {
      _images.pop_front();
    }
    lk.unlock();
    _images_con.notify_all();
  }

  void PushBackFuture(std::future<Image::UniquePtr> f)
  {
    std::unique_lock<std::mutex> lk(_futures_mut);
    _futures.emplace_back(std::move(f));
    lk.unlock();
    _futures_con.notify_one();
  }

private:
  void _Manager()
  {
    while (rclcpp::ok()) {
      std::unique_lock<std::mutex> lk(_futures_mut);
      if (_futures.empty() == false) {
        auto f = std::move(_futures.front());
        _futures.pop_front();
        lk.unlock();
        auto ptr = f.get();
        _node->Publish(ptr);
      } else {
        _futures_con.wait(lk);
      }
    }
  }

  void _Worker()
  {
    Image::_data_type _buf;
    while (rclcpp::ok()) {
      std::unique_lock<std::mutex> lk(_images_mut);
      if (_images.empty() == false) {
        auto ptr = std::move(_images.front());
        _images.pop_front();
        std::promise<Image::UniquePtr> prom;
        PushBackFuture(prom.get_future());
        lk.unlock();
        if (ptr->header.frame_id == "-1") {
          auto img = std::make_unique<Image>();
          img->header = ptr->header;
          prom.set_value(std::move(ptr));
        } else {
          _buf.resize(ptr->width * ptr->height / 4);
          cv::Mat dst(ptr->width / 2, ptr->height / 2, CV_8UC1, _buf.data());
          cv::Mat img(ptr->height, ptr->width, CV_8UC1, ptr->data.data());
          cv::resize(img, dst, 0.5, 0.5);
          std::swap(ptr->data, _buf);
          // std::swap(ptr->width, ptr->height);
          ptr->width /= 2;
          ptr->height /= 2;
          ptr->step /= 2;
          prom.set_value(std::move(ptr));
        }
      } else {
        _images_con.wait(lk);
      }
    }
  }

private:
  ResizeImage * _node;
  int _workers = 1;
  int _deque_size;

  std::mutex _images_mut;
  std::condition_variable _images_con;
  std::deque<Image::UniquePtr> _images;

  std::mutex _futures_mut;
  std::condition_variable _futures_con;
  std::deque<std::future<Image::UniquePtr>> _futures;

  std::vector<std::thread> _threads;
};

ResizeImage::ResizeImage(const rclcpp::NodeOptions & options)
: Node("resize_image_node", options)
{
  _pubImage = this->create_publisher<Image>(_pubImageName, rclcpp::SensorDataQoS());

  _impl = std::make_unique<_Impl>(this);

  _sub = this->create_subscription<Image>(
    _subName,
    rclcpp::SensorDataQoS(),
    [this](Image::UniquePtr ptr)
    {
      _impl->PushBackImage(std::move(ptr));
    }
  );

  RCLCPP_INFO(this->get_logger(), "Ininitialized successfully");
}

ResizeImage::~ResizeImage()
{
  _sub.reset();
  _impl.reset();
  _pubImage.reset();

  RCLCPP_INFO(this->get_logger(), "Destroyed successfully");
}

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(resize_image::ResizeImage)
