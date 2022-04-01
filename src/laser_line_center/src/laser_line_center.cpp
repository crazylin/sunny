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
#include <string>
#include <utility>
#include <vector>

#include "opencv2/opencv.hpp"

namespace laser_line_center
{

using sensor_msgs::msg::Image;
using sensor_msgs::msg::PointCloud2;
using sensor_msgs::msg::PointField;

const std::vector<std::string> KEYS = {"ksize", "threshold", "width_min", "width_max"};

const std::map<int, double> SCALAR = {
  {1, 1.},
  {3, 1. / 4.},
  {5, 1. / 48.},
  {7, 1. / 640.},
  {-1, 1. / 16.},
};

struct Params
{
  Params(LaserLineCenter * node)
  {
    const auto & vp = node->get_parameters(KEYS);
    for (const auto & p : vp) {
      if (p.get_name() == "ksize") {
        ksize = p.as_int();
      } else if (p.get_name() == "threshold") {
        threshold = p.as_int();
      } else if (p.get_name() == "width_min") {
        width_min = p.as_int();
      } else if (p.get_name() == "width_max") {
        width_max = p.as_int();
      }
    }
  }

  int ksize;
  int threshold;
  int width_min;
  int width_max;
};

class LaserLineCenter::_Impl
{
public:
  explicit _Impl(LaserLineCenter * ptr, int w)
  : _node(ptr), _workers(w)
  {
    declare_parameters();
    for (int i = 0; i < w; ++i) {
      _threads.push_back(std::thread(&_Impl::worker, this));
    }
    _threads.push_back(std::thread(&_Impl::manager, this));
    RCLCPP_INFO(_node->get_logger(), "Employ %d workers successfully", w);
  }

  ~_Impl()
  {
    _images_con.notify_all();
    _futures_con.notify_one();
    for (auto & t : _threads) {
      t.join();
    }
  }

  void declare_parameters()
  {
    _node->declare_parameter("ksize", 5);
    _node->declare_parameter("threshold", 35);
    _node->declare_parameter("width_min", 1);
    _node->declare_parameter("width_max", 30);
  }

  void push_back_image(Image::UniquePtr & ptr)
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

  void push_back_future(std::future<PointCloud2::UniquePtr> f)
  {
    std::unique_lock<std::mutex> lk(_futures_mut);
    _futures.emplace_back(std::move(f));
    lk.unlock();
    _futures_con.notify_one();
  }

  void manager()
  {
    while (rclcpp::ok()) {
      std::unique_lock<std::mutex> lk(_futures_mut);
      if (_futures.empty() == false) {
        auto f = std::move(_futures.front());
        _futures.pop_front();
        lk.unlock();
        auto ptr = f.get();
        _node->publish(ptr);
      } else {
        _futures_con.wait(lk);
      }
    }
  }

  void worker()
  {
    cv::Mat buf;
    while (rclcpp::ok()) {
      std::unique_lock<std::mutex> lk(_images_mut);
      if (_images.empty() == false) {
        auto ptr = std::move(_images.front());
        _images.pop_front();
        std::promise<PointCloud2::UniquePtr> prom;
        push_back_future(prom.get_future());
        auto pms = Params(_node);
        lk.unlock();
        if (ptr->header.frame_id == "-1" || ptr->data.empty()) {
          auto line = std::make_unique<PointCloud2>();
          line->header = ptr->header;
          prom.set_value(std::move(line));
        } else {
          cv::Mat img(ptr->height, ptr->width, CV_8UC1, ptr->data.data());
          auto line = execute(img, buf, pms);
          line->header = ptr->header;
          prom.set_value(std::move(line));
        }
      } else {
        _images_con.wait(lk);
      }
    }
  }

  PointCloud2::UniquePtr execute(const cv::Mat & img, cv::Mat & buf, const Params & pms)
  {
    std::vector<float> pnts;
    pnts.reserve(img.rows * 2);

    cv::Sobel(img, buf, CV_16S, 1, 0, pms.ksize, SCALAR.at(pms.ksize));
    for (decltype(img.rows) r = 0; r < img.rows; ++r) {
      auto pRow = buf.ptr<short>(r);  // NOLINT
      auto minmax = std::minmax_element(pRow, pRow + img.cols);

      auto minEle = minmax.first;
      auto maxEle = minmax.second;

      auto minVal = *minEle;
      auto maxVal = *maxEle;

      auto minPos = minEle - pRow;
      auto minP = minPos == 0 ? pRow[minPos + 1] : pRow[minPos - 1];
      auto minN = minPos == img.cols - 1 ? pRow[minPos - 1] : pRow[minPos + 1];

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
        maxVal > pms.threshold &&
        minVal < -pms.threshold &&
        width > pms.width_min &&
        width < pms.width_max &&
        maxPos > 0 &&
        minPos < img.cols - 1)
      {
        pnts.push_back(c);
      } else {
        pnts.push_back(-1.);
      }
    }

    return msgify(pnts);
  }

  PointCloud2::UniquePtr msgify(const std::vector<float> & pnts)
  {
    auto num = pnts.size();
    auto ptr = std::make_unique<PointCloud2>();

    ptr->height = 1;
    ptr->width = num;

    ptr->fields.resize(1);

    ptr->fields[0].name = "u";
    ptr->fields[0].offset = 0;
    ptr->fields[0].datatype = 7;
    ptr->fields[0].count = 1;

    ptr->is_bigendian = false;
    ptr->point_step = 4;
    ptr->row_step = num * 4;

    ptr->data.resize(num * 4);

    ptr->is_dense = true;

    memcpy(ptr->data.data(), pnts.data(), num * 4);

    return ptr;
  }

private:
  LaserLineCenter * _node;
  int _workers;

  std::mutex _images_mut;
  std::condition_variable _images_con;
  std::deque<Image::UniquePtr> _images;

  std::mutex _futures_mut;
  std::condition_variable _futures_con;
  std::deque<std::future<PointCloud2::UniquePtr>> _futures;

  std::vector<std::thread> _threads;
};

int workers(const rclcpp::NodeOptions & options)
{
  for (const auto & p : options.parameter_overrides()) {
    if (p.get_name() == "workers") {
      return p.as_int();
    }
  }
  return 1;
}

LaserLineCenter::LaserLineCenter(const rclcpp::NodeOptions & options)
: Node("laser_line_center_node", options)
{
  _pub = this->create_publisher<PointCloud2>(_pub_name, rclcpp::SensorDataQoS());

  _impl = std::make_unique<_Impl>(this, workers(options));

  _sub = this->create_subscription<Image>(
    _sub_name,
    rclcpp::SensorDataQoS(),
    [this](Image::UniquePtr ptr)
    {
      _impl->push_back_image(ptr);
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
