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

#include <deque>
#include <exception>
#include <memory>
#include <utility>
#include <vector>

#include "opencv2/opencv.hpp"

namespace rotate_image
{

using std_msgs::msg::Header;
using sensor_msgs::msg::Image;

class RotateImage::_Impl
{
public:
  explicit _Impl(RotateImage * ptr)
  : _node(ptr)
  {
    _node->declare_parameter("workers", _workers);
    _node->get_parameter("workers", _workers);
    for (int i = 0; i < _workers; ++i) {
      _threads.push_back(std::thread(&_Impl::_Worker, this));
    }
    RCLCPP_INFO(_node->get_logger(), "Employ %d workers successfully", _workers);
  }

  ~_Impl()
  {
    _con.notify_all();
    for (auto & t : _threads) {
      t.join();
    }
  }

  void PushBack(Image::UniquePtr ptr)
  {
    std::unique_lock<std::mutex> lk(_mutex);
    _deq.emplace_back(std::move(ptr));
    lk.unlock();
    _con.notify_all();
  }

private:
  void _Worker()
  {
    Image::_data_type _buf;
    while (rclcpp::ok()) {
      std::unique_lock<std::mutex> lk(_mutex);
      if (_deq.empty() == false) {
        if (_deq.size() == 60) {
          RCLCPP_WARN(_node->get_logger(), "Buffer exceed 60");
        }
        auto ptr = std::move(_deq.front());
        _deq.pop_front();
        lk.unlock();
        if (ptr->header.frame_id == "-1") {
          auto img = std::make_unique<Image>();
          img->header = ptr->header;
          _node->Publish(img);
        } else {
          if (ptr->encoding != "mono8") {
            RCLCPP_WARN(_node->get_logger(), "Can not handle color image");
            continue;
          } else {
            _buf.resize(ptr->width * ptr->height);
            cv::Mat dst(ptr->width, ptr->height, CV_8UC1, _buf.data());
            cv::Mat img(ptr->height, ptr->width, CV_8UC1, ptr->data.data());
            cv::rotate(img, dst, cv::ROTATE_90_CLOCKWISE);
            std::swap(ptr->data, _buf);
            std::swap(ptr->width, ptr->height);
            ptr->step = ptr->width;
            _node->Publish(ptr);
          }
        }
      } else {
        _con.wait(lk);
      }
    }
  }

private:
  RotateImage * _node;
  int _workers = 1;
  std::mutex _mutex;              ///< Mutex to protect shared storage
  std::condition_variable _con;   ///< Conditional variable rely on mutex
  std::deque<Image::UniquePtr> _deq;
  std::vector<std::thread> _threads;
};

RotateImage::RotateImage(const rclcpp::NodeOptions & options)
: Node("rotate_image_node", options)
{
  _pubImage = this->create_publisher<Image>(_pubImageName, rclcpp::SensorDataQoS());

  _pubHeader = this->create_publisher<Header>(_pubHeaderName, 10);

  _impl = std::make_unique<_Impl>(this);

  _sub = this->create_subscription<Image>(
    _subName,
    rclcpp::SensorDataQoS(),
    [this](Image::UniquePtr ptr)
    {
      _impl->PushBack(std::move(ptr));
    }
  );

  RCLCPP_INFO(this->get_logger(), "Ininitialized successfully");
}

RotateImage::~RotateImage()
{
  _sub.reset();
  _impl.reset();
  _pubHeader.reset();
  _pubImage.reset();

  RCLCPP_INFO(this->get_logger(), "Destroyed successfully");
}

}  // namespace rotate_image

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(rotate_image::RotateImage)
