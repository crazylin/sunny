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
#include <map>
#include <memory>
#include <utility>
#include <vector>

#include "opencv2/opencv.hpp"

namespace line_center_reconstruction
{

using std_msgs::msg::Header;
using sensor_msgs::msg::PointCloud2;
using sensor_msgs::msg::PointField;

class LineCenterReconstruction::_Impl
{
public:
  explicit _Impl(LineCenterReconstruction * ptr)
  : _node(ptr)
  {
    _InitializeParameters();
    _UpdateParameters();

    for (int i = 0; i < _workers; ++i) {
      _threads.push_back(std::thread(&LineCenterReconstruction::_Impl::_Worker, this));
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

  void PushBack(PointCloud2::UniquePtr & ptr)
  {
    std::unique_lock<std::mutex> lk(_mutex);
    _deq.emplace_back(std::move(ptr));
    lk.unlock();
    _con.notify_all();
  }

private:
  void _InitializeParameters()
  {
    std::vector<double> temp;
    _node->declare_parameter("camera_matrix", temp);
    _node->declare_parameter("distort_coeffs", temp);
    _node->declare_parameter("homography_matrix", temp);
    _node->declare_parameter("workers", _workers);
  }

  void _UpdateParameters()
  {
    std::vector<double> c, d, h;
    _node->get_parameter("camera_matrix", c);
    _node->get_parameter("distort_coeffs", d);
    _node->get_parameter("homography_matrix", h);
    _node->get_parameter("workers", _workers);

    _coef = cv::Mat(3, 3, CV_64F, c.data()).clone();
    _dist = cv::Mat(1, 5, CV_64F, d.data()).clone();
    _H = cv::Mat(3, 3, CV_64F, h.data()).clone();
  }

  void _Worker()
  {
    while (rclcpp::ok()) {
      std::unique_lock<std::mutex> lk(_mutex);
      if (_deq.empty() == false) {
        if (_deq.size() == 60) {
          RCLCPP_WARN(_node->get_logger(), "Buffer exceed 60");
        }
        auto ptr = std::move(_deq.front());
        _deq.pop_front();
        lk.unlock();
        _Execute(ptr);
      } else {
        _con.wait(lk);
      }
    }
  }

  void _Execute(PointCloud2::UniquePtr & ptr)
  {
    if (ptr->header.frame_id == "-1" || ptr->width == 0) {
      auto msg = std::make_unique<PointCloud2>();
      msg->header = ptr->header;
      _Publish(msg);
    } else {
      auto src = _FromPointCloud2(ptr);
      std::vector<cv::Point2f> dst;
      dst.reserve(ptr->width);
      cv::perspectiveTransform(src, dst, _H);
      auto msg = _ToPointCloud2(dst, src);
      msg->header = ptr->header;
      _Publish(msg);
    }
  }

  std::vector<cv::Point2f> _FromPointCloud2(const PointCloud2::UniquePtr & ptr)
  {
    auto num = ptr->width;
    std::vector<cv::Point2f> pnts;
    pnts.reserve(num);
    auto p = reinterpret_cast<float *>(ptr->data.data());
    for (size_t i = 0; i < num; ++i) {
      pnts.emplace_back(p[i * 2], p[i * 2 + 1]);
    }
    return pnts;
  }

  PointCloud2::UniquePtr _ToPointCloud2(
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

  void _Publish(PointCloud2::UniquePtr & ptr)
  {
    if (_workers == 1) {
      _node->Publish(ptr);
    } else {
      std::lock_guard<std::mutex> guard(_sync);
      auto id = std::stoi(ptr->header.frame_id);
      _buf[id] = std::move(ptr);
      if (_buf.size() > _workers * 2) {
        auto pos = _buf.begin();
        _node->Publish(pos->second);
        _buf.erase(pos);
      }
    }
  }

private:
  LineCenterReconstruction * _node;
  int _workers = 1;
  cv::Mat _coef, _dist, _H;
  std::map<int, PointCloud2::UniquePtr> _buf;
  std::mutex _mutex, _sync;       ///< Mutex to protect shared storage
  std::condition_variable _con;   ///< Conditional variable rely on mutex
  std::deque<PointCloud2::UniquePtr> _deq;
  std::vector<std::thread> _threads;
};

LineCenterReconstruction::LineCenterReconstruction(const rclcpp::NodeOptions & options)
: Node("line_center_reconstruction_node", options)
{
  _pub = this->create_publisher<PointCloud2>(_pubName, rclcpp::SensorDataQoS());

  _pubHeader = this->create_publisher<Header>(_pubHeaderName, 10);

  _impl = std::make_unique<_Impl>(this);

  _sub = this->create_subscription<PointCloud2>(
    _subName,
    rclcpp::SensorDataQoS(),
    [this](PointCloud2::UniquePtr ptr)
    {
      _impl->PushBack(ptr);
    }
  );

  RCLCPP_INFO(this->get_logger(), "Ininitialized successfully");
}

LineCenterReconstruction::~LineCenterReconstruction()
{
  _sub.reset();
  _impl.reset();
  _pubHeader.reset();
  _pub.reset();

  RCLCPP_INFO(this->get_logger(), "Destroyed successfully");
}

}  // namespace line_center_reconstruction

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(line_center_reconstruction::LineCenterReconstruction)
