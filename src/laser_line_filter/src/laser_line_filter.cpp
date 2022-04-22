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

// #include <deque>
// #include <exception>
// #include <future>
// #include <map>
// #include <memory>
// #include <string>
#include <utility>
#include <vector>

#include "impl/filter.hpp"

namespace laser_line_filter
{

PointCloud2::UniquePtr execute(PointCloud2::UniquePtr ptr, const Params & pm)
{
  if (ptr->header.frame_id == "-1" || ptr->data.empty() || pm.enable == false) {return ptr;}

  auto p = reinterpret_cast<float *>(ptr->data.data());
  auto width = static_cast<int>(ptr->width);
  filter(
    p,
    width,
    pm.window_size,
    pm.gap,
    pm.deviate,
    pm.step,
    pm.length);

  return ptr;
}

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

LaserLineFilter::LaserLineFilter(const rclcpp::NodeOptions & options)
: Node("laser_line_filter_node", options)
{
  _pub = this->create_publisher<PointCloud2>(_pub_name, rclcpp::SensorDataQoS());

  _declare_parameters();

  _workers = workers(options);
  for (int i = 0; i < _workers; ++i) {
    _threads.push_back(std::thread(&LaserLineFilter::_worker, this));
  }
  _threads.push_back(std::thread(&LaserLineFilter::_manager, this));

  _sub = this->create_subscription<PointCloud2>(
    _sub_name,
    rclcpp::SensorDataQoS(),
    [this](PointCloud2::UniquePtr ptr)
    {
      _push_back_point(std::move(ptr));
    }
  );

  _handle = this->add_on_set_parameters_callback(
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
    }
  );
  RCLCPP_INFO(this->get_logger(), "Ininitialized successfully");
}

LaserLineFilter::~LaserLineFilter()
{
  try {
    _sub.reset();
    _handle.reset();
    _points_con.notify_all();
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

void LaserLineFilter::_declare_parameters()
{
  Params pm;
  this->declare_parameter("enable", pm.enable);
  this->declare_parameter("window_size", pm.window_size);
  this->declare_parameter("gap", pm.gap);
  this->declare_parameter("deviate", pm.deviate);
  this->declare_parameter("step", pm.step);
  this->declare_parameter("length", pm.length);
}

Params LaserLineFilter::_update_parameters()
{
  Params pm;
  const auto & vp = this->get_parameters(KEYS);
  for (const auto & p : vp) {
    if (p.get_name() == "enable") {
      pm.enable = p.as_bool();
    } else if (p.get_name() == "window_size") {
      pm.window_size = p.as_int();
    } else if (p.get_name() == "gap") {
      pm.gap = p.as_int();
    } else if (p.get_name() == "deviate") {
      pm.deviate = p.as_double();
    } else if (p.get_name() == "step") {
      pm.step = p.as_double();
    } else if (p.get_name() == "length") {
      pm.length = p.as_int();
    }
  }
  return pm;
}

void LaserLineFilter::_worker()
{
  while (rclcpp::ok()) {
    std::unique_lock<std::mutex> lk(_points_mut);
    if (_points.empty() == false) {
      auto ptr = std::move(_points.front());
      _points.pop_front();
      std::promise<PointCloud2::UniquePtr> prom;
      _push_back_future(prom.get_future());
      auto pm = _update_parameters();
      lk.unlock();
      auto msg = execute(std::move(ptr), pm);
      prom.set_value(std::move(msg));
    } else {
      _points_con.wait(lk);
    }
  }
}

void LaserLineFilter::_manager()
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

void LaserLineFilter::_push_back_point(PointCloud2::UniquePtr ptr)
{
  std::unique_lock<std::mutex> lk(_points_mut);
  _points.emplace_back(std::move(ptr));
  auto s = static_cast<int>(_points.size());
  while (s > _workers + 1) {
    _points.pop_front();
  }
  lk.unlock();
  _points_con.notify_all();
}

void LaserLineFilter::_push_back_future(std::future<PointCloud2::UniquePtr> fut)
{
  std::unique_lock<std::mutex> lk(_futures_mut);
  _futures.emplace_back(std::move(fut));
  lk.unlock();
  _futures_con.notify_one();
}

}  // namespace laser_line_filter

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(laser_line_filter::LaserLineFilter)
