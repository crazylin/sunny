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
#include <string>
#include <utility>
#include <vector>

#include "opencv2/opencv.hpp"

namespace line_center_reconstruction
{

/**
 * @brief Construct a vector of points from ROS point cloud message.
 *
 * @param ptr Unique pointer to ROS point cloud.
 * @return std::vector<cv::Point2f> A vector of points.
 */
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

/**
 * @brief Construct ROS point cloud message from two vector of points with x, y, i information.
 *
 * @param dst A sequence of points represent x, y.
 * @param src A sequence of points represent u, v.
 * @return PointCloud2::UniquePtr Point cloud message to publish.
 */
PointCloud2::UniquePtr to_pc2(
  const std::vector<cv::Point2f> & dst,
  const std::vector<cv::Point2f> & src)
{
  auto num = dst.size();
  auto ptr = std::make_unique<PointCloud2>();

  ptr->height = 1;
  ptr->width = num;

  ptr->fields.resize(3);

  ptr->fields[0].name = "x";
  ptr->fields[0].offset = 0;
  ptr->fields[0].datatype = 7;
  ptr->fields[0].count = 1;

  ptr->fields[1].name = "y";
  ptr->fields[1].offset = 4;
  ptr->fields[1].datatype = 7;
  ptr->fields[1].count = 1;

  ptr->fields[2].name = "i";
  ptr->fields[2].offset = 8;
  ptr->fields[2].datatype = 7;
  ptr->fields[2].count = 1;

  ptr->is_bigendian = false;
  ptr->point_step = 4 * 3;
  ptr->row_step = 12 * num;

  ptr->data.resize(12 * num);

  ptr->is_dense = true;

  auto p = reinterpret_cast<float *>(ptr->data.data());
  for (size_t i = 0; i < num; ++i) {
    p[i * 3 + 0] = dst[i].x;
    p[i * 3 + 1] = dst[i].y;
    p[i * 3 + 2] = src[i].y;
  }

  return ptr;
}

/**
 * @brief The algorithm to transform points from image plane to laser plane.
 *
 * For more details of the algorithm, refer to the README.md.
 * @param ptr The input point cloud data.
 * @return PointCloud2::UniquePtr Point cloud message to publish.
 */
PointCloud2::UniquePtr execute(PointCloud2::UniquePtr ptr, const Params & pm)
{
  if (ptr->header.frame_id == "-1" || ptr->data.empty()) {return ptr;}

  auto _homo = cv::Mat(pm.homography_matrix, true).reshape(1, 3);
  auto src = from_pc2(ptr);
  std::vector<cv::Point2f> dst;
  dst.reserve(ptr->width);
  cv::perspectiveTransform(src, dst, _homo);
  auto msg = to_pc2(dst, src);
  msg->header = ptr->header;
  return msg;
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

LineCenterReconstruction::LineCenterReconstruction(const rclcpp::NodeOptions & options)
: Node("line_center_reconstruction_node", options)
{
  _pub = this->create_publisher<PointCloud2>(_pub_name, rclcpp::SensorDataQoS());

  _declare_parameters();

  _workers = workers(options);
  for (int i = 0; i < _workers; ++i) {
    _threads.push_back(std::thread(&LineCenterReconstruction::_worker, this));
  }
  _threads.push_back(std::thread(&LineCenterReconstruction::_manager, this));

  _sub = this->create_subscription<PointCloud2>(
    _sub_name,
    rclcpp::SensorDataQoS(),
    [this](PointCloud2::UniquePtr ptr)
    {
      _push_back_point(std::move(ptr));
    }
  );

  RCLCPP_INFO(this->get_logger(), "Ininitialized successfully");
}

LineCenterReconstruction::~LineCenterReconstruction()
{
  try {
    _sub.reset();
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

void LineCenterReconstruction::_declare_parameters()
{
  Params pm;
  this->declare_parameter("camera_matrix", pm.camera_matrix);
  this->declare_parameter("distort_coeffs", pm.distort_coeffs);
  this->declare_parameter("homography_matrix", pm.homography_matrix);
}

Params LineCenterReconstruction::_update_parameters()
{
  Params pm;
  const auto & vp = this->get_parameters(KEYS);
  for (const auto & p : vp) {
    if (p.get_name() == "camera_matrix") {
      pm.camera_matrix = p.as_double_array();
    } else if (p.get_name() == "distort_coeffs") {
      pm.distort_coeffs = p.as_double_array();
    } else if (p.get_name() == "homography_matrix") {
      pm.homography_matrix = p.as_double_array();
    }
  }
  return pm;
  // for (const auto & p : vp) {
  //   if (p.get_name() == "camera_matrix") {
  //     _coef = cv::Mat(p.as_double_array(), true).reshape(1, 3);
  //   } else if (p.get_name() == "distort_coeffs") {
  //     _dist = cv::Mat(p.as_double_array(), true).reshape(1, 1);
  //   } else if (p.get_name() == "homography_matrix") {
  //     _homo = cv::Mat(p.as_double_array(), true).reshape(1, 3);
  //   }
  // }
}

void LineCenterReconstruction::_worker()
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

void LineCenterReconstruction::_manager()
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

void LineCenterReconstruction::_push_back_point(PointCloud2::UniquePtr ptr)
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

void LineCenterReconstruction::_push_back_future(std::future<PointCloud2::UniquePtr> fut)
{
  std::unique_lock<std::mutex> lk(_futures_mut);
  _futures.emplace_back(std::move(fut));
  lk.unlock();
  _futures_con.notify_one();
}

}  // namespace line_center_reconstruction

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(line_center_reconstruction::LineCenterReconstruction)
