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
#include <string>
#include <utility>
#include <vector>

namespace laser_line_filter
{

using sensor_msgs::msg::PointCloud2;
using sensor_msgs::msg::PointField;

const std::vector<std::string> KEYS = {"enable", "window_size", "gap", "deviate", "step", "length"};

struct Params
{
  explicit Params(LaserLineFilter * node)
  {
    const auto & vp = node->get_parameters(KEYS);
    for (const auto & p : vp) {
      if (p.get_name() == "enable") {
        enable = p.as_bool();
      } else if (p.get_name() == "window_size") {
        ws = p.as_int();
      } else if (p.get_name() == "gap") {
        gap = p.as_int();
      } else if (p.get_name() == "deviate") {
        dev = p.as_double();
      } else if (p.get_name() == "step") {
        step = p.as_double();
      } else if (p.get_name() == "length") {
        length = p.as_int();
      }
    }
  }

  bool enable = false;
  int ws = 10;
  int gap = 5;
  double dev = 5.;
  double step = 2.;
  int length = 30;
};

class LaserLineFilter::_Impl
{
public:
  explicit _Impl(LaserLineFilter * ptr, int w)
  : _node(ptr), _workers(w)
  {
    declare_parameters();
    for (int i = 0; i < w; ++i) {
      _threads.push_back(std::thread(&_Impl::worker, this));
    }
    _threads.push_back(std::thread(&_Impl::manager, this));

    _handle = _node->add_on_set_parameters_callback(
      [this](const std::vector<rclcpp::Parameter> & vp) {
        rcl_interfaces::msg::SetParametersResult result;
        result.successful = true;
        for (const auto & p : vp) {
          if (p.get_name() == "window_size") {
            if (p.as_int() <= 0) {
              result.successful = false;
              result.reason = "Failed to set window size";
              return result;
            }
          } else if (p.get_name() == "gap") {
            if (p.as_int() <= 0) {
              result.successful = false;
              result.reason = "Failed to set gap";
              return result;
            }
          } else if (p.get_name() == "deviate") {
            if (p.as_double() <= 0) {
              result.successful = false;
              result.reason = "Failed to set deviate";
              return result;
            }
          } else if (p.get_name() == "step") {
            if (p.as_double() <= 0) {
              result.successful = false;
              result.reason = "Failed to set step";
              return result;
            }
          } else if (p.get_name() == "length") {
            if (p.as_int() <= 0) {
              result.successful = false;
              result.reason = "Failed to set length";
              return result;
            }
          }
        }
        return result;
      });

    RCLCPP_INFO(_node->get_logger(), "Employ %d workers successfully", w);
  }

  ~_Impl()
  {
    _handle.reset();
    _points_con.notify_all();
    _futures_con.notify_one();
    for (auto & t : _threads) {
      t.join();
    }
  }

  void declare_parameters()
  {
    _node->declare_parameter("enable", false);
    _node->declare_parameter("window_size", 10);
    _node->declare_parameter("gap", 5);
    _node->declare_parameter("deviate", 5.);
    _node->declare_parameter("step", 2.);
    _node->declare_parameter("length", 30);
  }

  void push_back_point(PointCloud2::UniquePtr & ptr)
  {
    std::unique_lock<std::mutex> lk(_points_mut);
    _points.emplace_back(std::move(ptr));
    auto s = static_cast<int>(_points.size());
    if (s > _workers + 1) {
      _points.pop_front();
    }
    lk.unlock();
    _points_con.notify_all();
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
    while (rclcpp::ok()) {
      std::unique_lock<std::mutex> lk(_points_mut);
      if (_points.empty() == false) {
        auto ptr = std::move(_points.front());
        _points.pop_front();
        std::promise<PointCloud2::UniquePtr> prom;
        push_back_future(prom.get_future());
        auto pms = Params(_node);
        lk.unlock();
        if (ptr->header.frame_id == "-1" || ptr->data.empty()) {
          prom.set_value(std::move(ptr));
        } else {
          auto msg = execute(std::move(ptr), pms);
          prom.set_value(std::move(msg));
        }
      } else {
        _points_con.wait(lk);
      }
    }
  }

  PointCloud2::UniquePtr execute(PointCloud2::UniquePtr ptr, const Params & pms)
  {
    if (pms.enable == false) {
      return ptr;
    }

    auto num = static_cast<int>(ptr->width);
    if (ptr->header.frame_id == "-1" || num == 0) {
      return ptr;
    } else {
      std::vector<float> buf;
      buf.resize(num, -1);
      auto p = reinterpret_cast<float *>(ptr->data.data());

      for (int i = pms.ws; i < num - pms.ws; ++i) {
        if (p[i] < 0) {
          continue;
        }

        float sum = 0;
        int hit = 0;
        for (auto j = -pms.ws; j <= pms.ws; ++j) {
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
        if (p[i] < 0 || buf[i] < 0) {
          continue;
        }
        if (abs(p[i] - buf[i]) > pms.dev) {
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
          if (j - f <= pms.gap && abs(p[j] - p[f]) / (j - f) < pms.step) {
            f = j;
            ++j;
          } else {
            break;
          }
        }
        if (f - i < pms.length) {
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
  int _workers;

  std::mutex _points_mut;
  std::condition_variable _points_con;
  std::deque<PointCloud2::UniquePtr> _points;

  std::mutex _futures_mut;
  std::condition_variable _futures_con;
  std::deque<std::future<PointCloud2::UniquePtr>> _futures;

  std::vector<std::thread> _threads;

  OnSetParametersCallbackHandle::SharedPtr _handle;
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

LaserLineFilter::LaserLineFilter(const rclcpp::NodeOptions & options)
: Node("laser_line_filter_node", options)
{
  _pub = this->create_publisher<PointCloud2>(_pub_name, rclcpp::SensorDataQoS());

  _impl = std::make_unique<_Impl>(this, workers(options));

  _sub = this->create_subscription<PointCloud2>(
    _sub_name,
    rclcpp::SensorDataQoS(),
    [this](PointCloud2::UniquePtr ptr)
    {
      _impl->push_back_point(ptr);
    }
  );

  RCLCPP_INFO(this->get_logger(), "Ininitialized successfully");
}

LaserLineFilter::~LaserLineFilter()
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

}  // namespace laser_line_filter

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(laser_line_filter::LaserLineFilter)
