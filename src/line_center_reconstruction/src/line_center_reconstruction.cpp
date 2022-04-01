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

#include "line_center_reconstruction/line_center_reconstruction.hpp"

#include <deque>
#include <exception>
#include <future>
#include <map>
#include <memory>
#include <utility>
#include <vector>

#include "opencv2/opencv.hpp"

namespace line_center_reconstruction
{

using sensor_msgs::msg::PointCloud2;
using sensor_msgs::msg::PointField;

const std::vector<std::string> KEYS = {"camera_matrix", "distort_coeffs", "homography_matrix"};

class LineCenterReconstruction::_Impl
{
public:
  explicit _Impl(LineCenterReconstruction * ptr, int w)
  : _node(ptr), _workers(w)
  {
    declare_parameters();
    get_parameters();

    for (int i = 0; i < w; ++i) {
      _threads.push_back(std::thread(&_Impl::worker, this));
    }
    _threads.push_back(std::thread(&_Impl::manager, this));
    RCLCPP_INFO(_node->get_logger(), "Employ %d workers successfully", w);
  }

  ~_Impl()
  {
    _points_con.notify_all();
    _futures_con.notify_one();
    for (auto & t : _threads) {
      t.join();
    }
  }

  void declare_parameters()
  {
    _node->declare_parameter<std::vector<double>>("camera_matrix");
    _node->declare_parameter<std::vector<double>>("distort_coeffs");
    _node->declare_parameter<std::vector<double>>("homography_matrix");
  }

  void get_parameters()
  {
    const auto & vp = _node->get_parameters(KEYS);
    for ( const auto & p : vp) {
      if (p.get_name() == "camera_matrix") {
        _coef = cv::Mat(p.as_double_array(), true).reshape(1, 3);
      } else if (p.get_name() == "distort_coeffs") {
        _dist = cv::Mat(p.as_double_array(), true).reshape(1, 1);
      } else if (p.get_name() == "homography_matrix") {
        _homo = cv::Mat(p.as_double_array(), true).reshape(1, 3);
      }
    }
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
        lk.unlock();
        auto msg = execute(ptr);
        prom.set_value(std::move(msg));
      } else {
        _points_con.wait(lk);
      }
    }
  }

  PointCloud2::UniquePtr execute(PointCloud2::UniquePtr & ptr)
  {
    if (ptr->header.frame_id == "-1" || ptr->data.empty()) {
      auto msg = std::make_unique<PointCloud2>();
      msg->header = ptr->header;
      return msg;
    } else {
      auto src = from_pc2(ptr);
      if (src.empty()) {
        auto msg = std::make_unique<PointCloud2>();
        msg->header = ptr->header;
        return msg;
      }
      std::vector<cv::Point2f> dst;
      dst.reserve(ptr->width);
      cv::perspectiveTransform(src, dst, _homo);
      auto msg = to_pc2(dst, src);
      msg->header = ptr->header;
      return msg;
    }
  }

  std::vector<cv::Point2f> from_pc2(const PointCloud2::UniquePtr & ptr)
  {
    auto num = ptr->width;
    std::vector<cv::Point2f> pnts;
    pnts.reserve(num);
    auto p = reinterpret_cast<float *>(ptr->data.data());
    for (size_t i = 0; i < num; ++i) {
      if (p[i] > 0) {
        pnts.emplace_back(p[i], static_cast<float>(i));
      }
    }
    return pnts;
  }

  PointCloud2::UniquePtr to_pc2(
    const std::vector<cv::Point2f> & dst,
    const std::vector<cv::Point2f> & src)
  {
    auto num = dst.size();
    auto ptr = std::make_unique<PointCloud2>();

    ptr->height = 1;
    ptr->width = num;

    ptr->fields.resize(4);

    ptr->fields[0].name = "x";
    ptr->fields[0].offset = 0;
    ptr->fields[0].datatype = 7;
    ptr->fields[0].count = 1;

    ptr->fields[1].name = "y";
    ptr->fields[1].offset = 4;
    ptr->fields[1].datatype = 7;
    ptr->fields[1].count = 1;

    ptr->fields[2].name = "u";
    ptr->fields[2].offset = 8;
    ptr->fields[2].datatype = 7;
    ptr->fields[2].count = 1;

    ptr->fields[3].name = "v";
    ptr->fields[3].offset = 12;
    ptr->fields[3].datatype = 7;
    ptr->fields[3].count = 1;

    ptr->is_bigendian = false;
    ptr->point_step = 4 * 4;
    ptr->row_step = 16 * num;

    ptr->data.resize(16 * num);

    ptr->is_dense = true;

    auto p = reinterpret_cast<float *>(ptr->data.data());
    for (size_t i = 0; i < num; ++i) {
      p[i * 4 + 0] = dst[i].x;
      p[i * 4 + 1] = dst[i].y;
      p[i * 4 + 2] = src[i].x;
      p[i * 4 + 3] = src[i].y;
    }

    return ptr;
  }

private:
  LineCenterReconstruction * _node;
  int _workers;

  cv::Mat _coef, _dist, _homo;

  std::mutex _points_mut;
  std::condition_variable _points_con;
  std::deque<PointCloud2::UniquePtr> _points;

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

LineCenterReconstruction::LineCenterReconstruction(const rclcpp::NodeOptions & options)
: Node("line_center_reconstruction_node", options)
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

LineCenterReconstruction::~LineCenterReconstruction()
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

}  // namespace line_center_reconstruction

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(line_center_reconstruction::LineCenterReconstruction)
