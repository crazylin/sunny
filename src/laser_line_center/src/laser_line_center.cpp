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

#include "laser_line_center/laser_line_center.hpp"

#include <deque>
#include <exception>
#include <map>
#include <memory>
#include <utility>

#include "opencv2/opencv.hpp"

namespace laser_line_center
{

using std_msgs::msg::Header;
using shared_interfaces::msg::LineCenter;
using sensor_msgs::msg::Image;

class LaserLineCenter::_Impl
{
public:
  explicit _Impl(LaserLineCenter * ptr)
  : _node(ptr)
  {
    _InitializeParameters();
    _UpdateParameters();
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

  void PushBack(Image::UniquePtr & ptr)
  {
    std::unique_lock<std::mutex> lk(_mutex);
    _deq.emplace_back(std::move(ptr));
    lk.unlock();
    _con.notify_all();
  }

private:
  void _InitializeParameters()
  {
    _node->declare_parameter("ksize", _ksize);
    _node->declare_parameter("threshold", _threshold);
    _node->declare_parameter("width_min", _widthMin);
    _node->declare_parameter("width_max", _widthMax);
    _node->declare_parameter("workers", _workers);
  }

  void _UpdateParameters()
  {
    _node->get_parameter("ksize", _ksize);
    _node->get_parameter("threshold", _threshold);
    _node->get_parameter("width_min", _widthMin);
    _node->get_parameter("width_max", _widthMax);
    _node->get_parameter("workers", _workers);
  }

  LineCenter::UniquePtr _Execute(const cv::Mat & img, cv::Mat & _dx)
  {
    auto line = std::make_unique<LineCenter>();
    line->center.resize(img.rows, -1);

    cv::Sobel(img, _dx, CV_16S, 1, 0, _ksize, _scale[_ksize]);

    for (decltype(img.rows) r = 0; r < img.rows; ++r) {
      auto pRow = _dx.ptr<short>(r);  // NOLINT
      auto minmax = std::minmax_element(pRow, pRow + img.cols);

      auto minEle = minmax.first;
      auto maxEle = minmax.second;

      auto minVal = *minEle;
      auto maxVal = *maxEle;

      auto minPos = minEle - pRow;
      auto maxPos = maxEle - pRow;
      auto width = minPos - maxPos;

      // Filter by threshold
      if (maxVal < _threshold || minVal > -_threshold) {continue;}

      // Filter by width
      if (width < _widthMin || width > _widthMax) {continue;}

      // Filter by position
      if (maxPos <= 0 || minPos >= img.cols - 1) {continue;}

      auto a1 = pRow[maxPos - 1] + pRow[maxPos + 1] - pRow[maxPos] * 2;
      auto b1 = pRow[maxPos + 1] - pRow[maxPos - 1];
      auto s1 = -0.5 * b1 / a1;

      auto a2 = pRow[minPos - 1] + pRow[minPos + 1] - pRow[minPos] * 2;
      auto b2 = pRow[minPos + 1] - pRow[minPos - 1];
      auto s2 = -0.5 * b2 / a2;

      line->center[r] = (maxPos + minPos + s1 + s2) * 0.5;
    }

    return line;
  }

  void _Worker()
  {
    cv::Mat _dx;
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
          auto line = std::make_unique<LineCenter>();
          line->header = ptr->header;
          _node->Publish(ptr->header);
          _node->Publish(line);
        } else {
          if (ptr->encoding != "mono8") {
            RCLCPP_WARN(_node->get_logger(), "Can not handle color image");
            continue;
          }

          cv::Mat img(ptr->height, ptr->width, CV_8UC1, ptr->data.data());
          if (img.empty()) {
            RCLCPP_WARN(_node->get_logger(), "Image message is empty");
            continue;
          }

          _UpdateParameters();

          auto line = _Execute(img, _dx);
          line->header = ptr->header;
          _node->Publish(ptr->header);
          _node->Publish(line);
        }
      } else {
        _con.wait(lk);
      }
    }
  }

private:
  int _ksize = 5;
  int _threshold = 35;
  int _widthMin = 1;
  int _widthMax = 30;
  int _workers = 1;
  std::map<int, double> _scale = {
    {1, 1.},
    {3, 1. / 4.},
    {5, 1. / 48.},
    {7, 1. / 640.},
    {-1, 1. / 16.}
  };

  LaserLineCenter * _node;
  std::mutex _mutex;              ///< Mutex to protect shared storage
  std::condition_variable _con;   ///< Conditional variable rely on mutex
  std::deque<Image::UniquePtr> _deq;
  std::vector<std::thread> _threads;
};

LaserLineCenter::LaserLineCenter(const rclcpp::NodeOptions & options)
: Node("laser_line_center_node", options)
{
  _pub = this->create_publisher<LineCenter>(_pubName, rclcpp::SensorDataQoS());

  _pubHeader = this->create_publisher<Header>(_pubHeaderName, 10);

  _impl = std::make_unique<_Impl>(this);

  _sub = this->create_subscription<Image>(
    _subName,
    rclcpp::SensorDataQoS(),
    [this](Image::UniquePtr ptr)
    {
      _impl->PushBack(ptr);
    }
  );

  RCLCPP_INFO(this->get_logger(), "Ininitialized successfully");
}

LaserLineCenter::~LaserLineCenter()
{
  try {
    _sub.reset();
    _impl.reset();
    _pubHeader.reset();
    _pub.reset();

    RCLCPP_INFO(this->get_logger(), "Destroyed successfully");
  } catch (const std::exception & e) {
    RCLCPP_ERROR(this->get_logger(), "Exception in destructor: %s", e.what());
  } catch (...) {
    RCLCPP_ERROR(this->get_logger(), "Exception in destructor: unknown");
  }
}

}  // namespace laser_line_center

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(laser_line_center::LaserLineCenter)
