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

#include "laser_line_filter/laser_line_filter.hpp"

#include <deque>
#include <exception>
#include <future>
#include <map>
#include <memory>
#include <utility>
#include <vector>

namespace laser_line_filter
{

using sensor_msgs::msg::PointCloud2;
using sensor_msgs::msg::PointField;

class LaserLineFilter::_Impl
{
public:
  explicit _Impl(LaserLineFilter * ptr)
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
    _points_con.notify_all();
    _futures_con.notify_one();
    for (auto & t : _threads) {
      t.join();
    }
  }

  void PushBackPoint(PointCloud2::UniquePtr & ptr)
  {
    std::unique_lock<std::mutex> lk(_points_mut);
    _points.emplace_back(std::move(ptr));
    auto s = static_cast<int>(_points.size());
    if (s > _deque_size) {
      _points.pop_front();
    }
    lk.unlock();
    _points_con.notify_all();
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
    _node->declare_parameter("workers", 1);
    _node->declare_parameter("window_size", 10);
    _node->declare_parameter("deviate", 5.);
    _node->declare_parameter("step", 2.);
    _node->declare_parameter("length", 30);
  }

  void _UpdateParameters()
  {
    _node->get_parameter("workers", _workers);
  }

  void _GetParameters(int & ws, double & dev, double & step, int & length)
  {
    auto vp = _node->get_parameters({"window_size", "deviate", "step", "length"});
    ws = vp[0].as_int();
    dev = vp[1].as_double();
    step = vp[2].as_double();
    length = vp[3].as_int();
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
    while (rclcpp::ok()) {
      std::unique_lock<std::mutex> lk(_points_mut);
      if (_points.empty() == false) {
        auto ptr = std::move(_points.front());
        _points.pop_front();
        std::promise<PointCloud2::UniquePtr> prom;
        PushBackFuture(prom.get_future());
        int ws, length;
        double dev, step;
        _GetParameters(ws, dev, step, length);
        lk.unlock();
        auto msg = _Execute(std::move(ptr), ws, dev, step, length);
        prom.set_value(std::move(msg));
      } else {
        _points_con.wait(lk);
      }
    }
  }

  PointCloud2::UniquePtr _Execute(PointCloud2::UniquePtr ptr, int ws, double dev, double step, int length)
  {
    auto num = static_cast<int>(ptr->width);
    if (ptr->header.frame_id == "-1" || num == 0) {
      return ptr;
    } else {
      std::vector<float> buf;
      buf.resize(num, -1);
      auto p = reinterpret_cast<float *>(ptr->data.data());

      for (int i = ws; i < num - ws; ++i) { // window
        if (p[i] < 0) {
          continue;
        }

        float sum = 0;
        int hit = 0;
        for (auto j = -ws; j <= ws; ++j) { // window
          if (p[i + j] < 0) {
            continue;
          }
          sum += p[i + j];
          ++hit;
        }
        buf[i] = sum / hit;
      }
      
      // filter by diff with average
      for (int i = 0; i < num; ++i) {
        if (p[i] < 0) {
          continue;
        }
        if (abs(p[i] - buf[i]) > dev) { // diff
          p[i] = -1;
        }
      }

      // filter by length
      auto i = 0;
      while (i < num) {
        if (p[i] < 0) {
          ++i;
          continue;
        }
        auto f = i;
        auto j = f + 1;
        while (j < num) {
          if (p[j] < 0) {
            ++j;
            continue;
          }
          if (abs(p[j] - p[f]) / (j - f) < step) { // step
            f = j;
            ++j;
          }
          else {
            break;
          }
        }
        if (f - i < length) { // width
          for (auto k = i; k <= f; ++k) {
            p[k] = -1;
          }
        } else {
          i = j;
        }
      }

      return ptr;
    }
  }

private:
  LaserLineFilter * _node;
  int _workers = 1;
  int _deque_size;

  std::mutex _points_mut;
  std::condition_variable _points_con;
  std::deque<PointCloud2::UniquePtr> _points;

  std::mutex _futures_mut;
  std::condition_variable _futures_con;
  std::deque<std::future<PointCloud2::UniquePtr>> _futures;

  std::vector<std::thread> _threads;
};

LaserLineFilter::LaserLineFilter(const rclcpp::NodeOptions & options)
: Node("laser_line_filter_node", options)
{
  _pub = this->create_publisher<PointCloud2>(_pubName, rclcpp::SensorDataQoS());

  _impl = std::make_unique<_Impl>(this);

  _sub = this->create_subscription<PointCloud2>(
    _subName,
    rclcpp::SensorDataQoS(),
    [this](PointCloud2::UniquePtr ptr)
    {
      _impl->PushBackPoint(ptr);
    }
  );

  this->add_on_set_parameters_callback(
   [this](const std::vector<rclcpp::Parameter> & parameters) {
      rcl_interfaces::msg::SetParametersResult result;
      result.successful = true;
      for (const auto & parameter : parameters) {
        if (parameter.get_name() == "window_size") {
          if (parameter.as_int() <= 0) {
            result.successful = false;
            result.reason = "Failed to set window size";
          }
        } else if (parameter.get_name() == "deviate") {
          if (parameter.as_double() <= 0) {
            result.successful = false;
            result.reason = "Failed to set deviate";
          }
        } else if (parameter.get_name() == "step") {
          if (parameter.as_double() <= 0) {
            result.successful = false;
            result.reason = "Failed to set step";
          }
        } else if (parameter.get_name() == "length") {
          if (parameter.as_int() <= 0) {
            result.successful = false;
            result.reason = "Failed to set length";
          }
        }
      }
      return result;
    });

  RCLCPP_INFO(this->get_logger(), "Ininitialized successfully");
}

LaserLineFilter::~LaserLineFilter()
{
  _sub.reset();
  _impl.reset();
  _pub.reset();

  RCLCPP_INFO(this->get_logger(), "Destroyed successfully");
}

}  // namespace laser_line_filter

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(laser_line_filter::LaserLineFilter)
