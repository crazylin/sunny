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

#include "impl/params.hpp"
#include "impl/filter.hpp"

namespace laser_line_filter
{

using sensor_msgs::msg::PointCloud2;
using sensor_msgs::msg::PointField;

/**
 * @brief Inner implementation for the algorithm.
 *
 */
class LaserLineFilter::_Impl
{
public:
  /**
   * @brief Construct a new impl object.
   *
   * Declare parameters before usage.
   * Create a thread for each worker.
   * Create a thread for manager.
   * Initialize ROS parameter callback.
   * Print success if all done.
   * @param ptr Reference to parent node.
   * @param w Number of workers to process simultaneously.
   */
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

  /**
   * @brief Destroy the impl object.
   *
   * Wake up all workers.
   * Wake up the manager.
   * Synchronize with all threads, wait for its return.
   */
  ~_Impl()
  {
    _handle.reset();
    _points_con.notify_all();
    _futures_con.notify_one();
    for (auto & t : _threads) {
      t.join();
    }
  }

  /**
   * @brief Declare parameters with defaults before usage.
   *
   */
  void declare_parameters()
  {
    _node->declare_parameter("enable", false);
    _node->declare_parameter("window_size", 10);
    _node->declare_parameter("gap", 5);
    _node->declare_parameter("deviate", 5.);
    _node->declare_parameter("step", 2.);
    _node->declare_parameter("length", 30);
  }

  /**
   * @brief Get parameters from ROS
   * 
   * @return Params 
   */
  Params update_parameters()
  {
    Params pm;
    const auto & vp = _node->get_parameters(KEYS);
    for (const auto & p : vp) {
      if (p.get_name() == "enable") {
        pm.enable = p.as_bool();
      } else if (p.get_name() == "window_size") {
        pm.ws = p.as_int();
      } else if (p.get_name() == "gap") {
        pm.gap = p.as_int();
      } else if (p.get_name() == "deviate") {
        pm.dev = p.as_double();
      } else if (p.get_name() == "step") {
        pm.step = p.as_double();
      } else if (p.get_name() == "length") {
        pm.length = p.as_int();
      }
    }
    return pm;
  }

  /**
   * @brief Push a point cloud and notity workers.
   *
   * @param ptr Reference to a unique pointer to point clout to be moved.
   */
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

  /**
   * @brief Promise a future so its future can be sychronized and notify the manager.
   *
   * @param f A future to point cloud msg.
   */
  void push_back_future(std::future<PointCloud2::UniquePtr> f)
  {
    std::unique_lock<std::mutex> lk(_futures_mut);
    _futures.emplace_back(std::move(f));
    lk.unlock();
    _futures_con.notify_one();
  }

  /**
   * @brief The manager works in seperate thread to gather worker's results in order.
   *
   * Spin infinitely until rclcpp:ok() return false.
   * Whenever a future is ready, the manager wake up, get the result from the future and publish.
   */
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

  /**
   * @brief The worker works in seperate thread to process incoming date parallelly.
   *
   * Enter infinite loop.
   * Wait for incoming data.
   * Wake up to get a possible data, make a promise and notify the manager.
   * Continue to work on the data and return to sleep if no further data to process.
   */
  void worker()
  {
    while (rclcpp::ok()) {
      std::unique_lock<std::mutex> lk(_points_mut);
      if (_points.empty() == false) {
        auto ptr = std::move(_points.front());
        _points.pop_front();
        std::promise<PointCloud2::UniquePtr> prom;
        push_back_future(prom.get_future());
        auto pms = update_parameters();
        lk.unlock();
        auto msg = filter(std::move(ptr), pms);
        prom.set_value(std::move(msg));
      } else {
        _points_con.wait(lk);
      }
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

/**
 * @brief Construct a new Laser Line Filter object.
 *
 * Initialize publisher.
 * Create an inner implementation.
 * Initialize subscription.
 * Print success if all done.
 * @param options Encapsulation of options for node initialization.
 */
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

/**
 * @brief Destroy the Laser Line Filter object.
 *
 * Release subscription.
 * Release inner implementation.
 * Release publisher.
 * Print success if all done.
 * Throw no exception.
 */
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
