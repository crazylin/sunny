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
#include <future>
#include <map>
#include <memory>
#include <utility>
#include <vector>

#include "opencv2/opencv.hpp"

namespace laser_line_center
{

using sensor_msgs::msg::Image;
using sensor_msgs::msg::PointCloud2;
using sensor_msgs::msg::PointField;

class LaserLineCenter::_Impl
{
public:
  explicit _Impl(LaserLineCenter * ptr)
  : _node(ptr)
  {
    _InitializeParameters();
    _UpdateParameters();
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

  void PushBackImage(Image::UniquePtr & ptr)
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

  void PushBackFuture(std::future<PointCloud2::UniquePtr> f)
  {
    std::unique_lock<std::mutex> lk(_futures_mut);
    _futures.emplace_back(std::move(f));
    lk.unlock();
    _futures_con.notify_one();
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
    cv::Mat _dx;
    while (rclcpp::ok()) {
      std::unique_lock<std::mutex> lk(_images_mut);
      if (_images.empty() == false) {
        auto ptr = std::move(_images.front());
        _images.pop_front();
        std::promise<PointCloud2::UniquePtr> prom;
        PushBackFuture(prom.get_future());
        lk.unlock();
        if (ptr->header.frame_id == "-1") {
          auto line = std::make_unique<PointCloud2>();
          line->header = ptr->header;
          prom.set_value(std::move(line));
        } else {
          cv::Mat img(ptr->height, ptr->width, CV_8UC1, ptr->data.data());
          if (img.empty()) {
            RCLCPP_WARN(_node->get_logger(), "Image message is empty");
            auto line = std::make_unique<PointCloud2>();
            line->header = ptr->header;
            prom.set_value(std::move(line));
          } else {
            _UpdateParameters();
            auto line = _Execute(img, _dx);
            line->header = ptr->header;
            prom.set_value(std::move(line));
          }
        }
      } else {
        _images_con.wait(lk);
      }
    }
  }

  PointCloud2::UniquePtr _Execute(const cv::Mat & img, cv::Mat & _dx)
  {
    std::vector<float> pnts;
    pnts.reserve(img.rows * 2);

    cv::Sobel(img, _dx, CV_16S, 1, 0, _ksize, _scale[_ksize]);
    for (decltype(img.rows) r = 0; r < img.rows; ++r) {
      auto pRow = _dx.ptr<short>(r);  // NOLINT
      auto minmax = std::minmax_element(pRow, pRow + img.cols);

      auto minEle = minmax.first;
      auto maxEle = minmax.second;

      auto minVal = *minEle;
      auto maxVal = *maxEle;

      auto minPos = minEle - pRow;
      auto minP = minPos == 0 ? pRow[minPos + 1] : pRow[minPos - 1];
      auto minN = minPos == img.cols - 1 : pRow[minPos - 1] : pRow[minPos + 1];

      auto maxPos = maxEle - pRow;
      auto maxP = maxPos == 0 ? pRow[maxPos + 1] : pRow[maxPos - 1];
      auto maxN = maxPos == img.cols - 1 ? pRow[maxPos - 1] : pRow[maxPos + 1];

      auto width = minPos - maxPos;

      auto a1 = maxP + maxN - maxVal * 2;
      auto b1 = maxP - maxN;
      auto s1 = (a1 < 0 ? 0.5 * b1 / a1 : 0.5 * b1);

      auto a2 = minP + minN - minVal * 2;
      auto b2 = minP - minN;
      auto s2 = (a2 > 0 ? 0.5 * b2 / a2 : 0.5 * b2);

      auto c = (maxPos + minPos + s1 + s2) / 2.;

      if (
        maxVal > _threshold &&
        minVal < -_threshold &&
        width > _widthMin &&
        width < _widthMax &&
        maxPos > 0 &&
        minPos < img.cols - 1)
      {
        pnts.push_back(c);
        pnts.push_back(r);
      }
    }

    return _ConstructPointCloud2(pnts);
  }

  PointCloud2::UniquePtr _ConstructPointCloud2(const std::vector<float> & pnts)
  {
    auto num = pnts.size() / 2;
    auto ptr = std::make_unique<PointCloud2>();

    ptr->height = 1;
    ptr->width = num;

    ptr->fields.resize(2);

    ptr->fields[0].name = "u";
    ptr->fields[0].offset = 0;
    ptr->fields[0].datatype = 7;
    ptr->fields[0].count = 1;

    ptr->fields[1].name = "v";
    ptr->fields[1].offset = 4;
    ptr->fields[1].datatype = 7;
    ptr->fields[1].count = 1;

    ptr->is_bigendian = false;
    ptr->point_step = 8;
    ptr->row_step = 8 * num;

    ptr->data.resize(8 * num);

    ptr->is_dense = true;

    memcpy(ptr->data.data(), pnts.data(), 8 * num);

    return ptr;
  }

private:
  int _ksize = 5;
  int _threshold = 35;
  int _widthMin = 1;
  int _widthMax = 30;
  int _workers = 1;
  int _deque_size;
  std::map<int, double> _scale = {
    {1, 1.},
    {3, 1. / 4.},
    {5, 1. / 48.},
    {7, 1. / 640.},
    {-1, 1. / 16.}
  };

  LaserLineCenter * _node;

  std::mutex _images_mut;
  std::condition_variable _images_con;
  std::deque<Image::UniquePtr> _images;

  std::mutex _futures_mut;
  std::condition_variable _futures_con;
  std::deque<std::future<PointCloud2::UniquePtr>> _futures;

  std::vector<std::thread> _threads;
};

LaserLineCenter::LaserLineCenter(const rclcpp::NodeOptions & options)
: Node("laser_line_center_node", options)
{
  _pub = this->create_publisher<PointCloud2>(_pubName, rclcpp::SensorDataQoS());

  _impl = std::make_unique<_Impl>(this);

  _sub = this->create_subscription<Image>(
    _subName,
    rclcpp::SensorDataQoS(),
    [this](Image::UniquePtr ptr)
    {
      _impl->PushBackImage(ptr);
    }
  );

  RCLCPP_INFO(this->get_logger(), "Ininitialized successfully");
}

LaserLineCenter::~LaserLineCenter()
{
  try {
    _sub.reset();
    _impl.reset();
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
