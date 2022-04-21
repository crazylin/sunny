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

#include <memory>
#include <utility>
#include <vector>

#include "opencv2/opencv.hpp"
#include "impl/center.hpp"

namespace laser_line_center
{

/**
 * @brief Construct ROS point cloud message from vector of floats.
 *
 * @param pnts A sequence of floats as points' row coordinate.
 * @return PointCloud2::UniquePtr Point cloud message to publish.
 */
PointCloud2::UniquePtr to_pc2(const std::vector<float> & pnts)
{
  auto ptr = std::make_unique<PointCloud2>();
  if (pnts.empty()) {return ptr;}

  auto num = pnts.size();

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

/**
 * @brief Given an image, a point cloud is returned.
 *
 * @param ptr ROS image
 * @param buf Buffer for sobel
 * @param pm Zipped input parameters
 * @return PointCloud2::UniquePtr ROS point cloud
 */
PointCloud2::UniquePtr execute(Image::UniquePtr ptr, cv::Mat & buf, const Params & pm)
{
  if (ptr->header.frame_id == "-1" || ptr->data.empty()) {
    auto line = std::make_unique<PointCloud2>();
    line->header = ptr->header;
    return line;
  }
  cv::Mat img(ptr->height, ptr->width, CV_8UC1, ptr->data.data());
  auto pnts = center(img, buf, pm.ksize, pm.threshold, pm.width_min, pm.width_max);
  auto line = to_pc2(pnts);
  line->header = ptr->header;
  return line;
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

LaserLineCenter::LaserLineCenter(const rclcpp::NodeOptions & options)
: Node("laser_line_center_node", options)
{
  _pub = this->create_publisher<PointCloud2>(_pub_name, rclcpp::SensorDataQoS());

  _declare_parameters();

  auto w = workers(options);
  for (int i = 0; i < w; ++i) {
    _threads.push_back(std::thread(&LaserLineCenter::_worker, this));
  }
  _threads.push_back(std::thread(&LaserLineCenter::_manager, this));

  _sub = this->create_subscription<Image>(
    _sub_name,
    rclcpp::SensorDataQoS(),
    [this](Image::UniquePtr ptr)
    {
      _push_back_image(std::move(ptr));
    }
  );

  _handle = this->add_on_set_parameters_callback(
    [this](const std::vector<rclcpp::Parameter> & vp) {
      rcl_interfaces::msg::SetParametersResult result;
      result.successful = true;
      for (const auto & p : vp) {
        if (p.get_name() == "ksize") {
          auto k = p.as_int();
          if (k != 1 && k != 3 && k != 5 && k != 7 && k != -1) {
            result.successful = false;
            result.reason = "Failed to set ksize [1, 3, 5, 7, -1]";
            return result;
          }
        } else if (p.get_name() == "threshold") {
          if (p.as_int() <= 0) {
            result.successful = false;
            result.reason = "Failed to set threshold";
            return result;
          }
        } else if (p.get_name() == "width_min") {
          if (p.as_double() <= 0) {
            result.successful = false;
            result.reason = "Failed to set width_min";
            return result;
          }
        } else if (p.get_name() == "width_max") {
          if (p.as_double() <= 0) {
            result.successful = false;
            result.reason = "Failed to set width_max";
            return result;
          }
        }
      }
      return result;
    }
  );

  RCLCPP_INFO(this->get_logger(), "Ininitialized successfully");
}

LaserLineCenter::~LaserLineCenter()
{
  try {
    _sub.reset();
    _images_con.notify_all();
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

void LaserLineCenter::_declare_parameters()
{
  Params pm;
  this->declare_parameter("ksize", pm.ksize);
  this->declare_parameter("threshold", pm.threshold);
  this->declare_parameter("width_min", pm.width_min);
  this->declare_parameter("width_max", pm.width_max);
}

Params LaserLineCenter::_update_parameters()
{
  Params pm;
  const auto & vp = this->get_parameters(KEYS);
  for (const auto & p : vp) {
    if (p.get_name() == "ksize") {
      pm.ksize = p.as_int();
    } else if (p.get_name() == "threshold") {
      pm.threshold = p.as_int();
    } else if (p.get_name() == "width_min") {
      pm.width_min = p.as_double();
    } else if (p.get_name() == "width_max") {
      pm.width_max = p.as_double();
    }
  }
  return pm;
}

void LaserLineCenter::_worker()
{
  cv::Mat buf;
  while (rclcpp::ok()) {
    std::unique_lock<std::mutex> lk(_images_mut);
    if (_images.empty() == false) {
      auto ptr = std::move(_images.front());
      _images.pop_front();
      std::promise<PointCloud2::UniquePtr> prom;
      _push_back_future(prom.get_future());
      auto pm = _update_parameters();
      lk.unlock();
      auto line = execute(std::move(ptr), buf, pm);
      prom.set_value(std::move(line));
    } else {
      _images_con.wait(lk);
    }
  }
}

void LaserLineCenter::_manager()
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

void LaserLineCenter::_push_back_image(Image::UniquePtr ptr)
{
  std::unique_lock<std::mutex> lk(_images_mut);
  _images.emplace_back(std::move(ptr));
  auto s = static_cast<int>(_images.size());
  while (s > _workers + 1) {
    _images.pop_front();
  }
  lk.unlock();
  _images_con.notify_all();
}

void LaserLineCenter::_push_back_future(std::future<PointCloud2::UniquePtr> fut)
{
  std::unique_lock<std::mutex> lk(_futures_mut);
  _futures.emplace_back(std::move(fut));
  lk.unlock();
  _futures_con.notify_one();
}

}  // namespace laser_line_center

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(laser_line_center::LaserLineCenter)
