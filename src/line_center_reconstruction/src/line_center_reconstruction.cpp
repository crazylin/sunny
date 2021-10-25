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
#include <memory>
#include <utility>
#include <vector>

#include "opencv2/opencv.hpp"

namespace line_center_reconstruction
{

using shared_interfaces::msg::LineCenter;
using sensor_msgs::msg::PointCloud2;

class LineCenterReconstruction::_Impl
{
public:
  explicit _Impl(LineCenterReconstruction * ptr)
  : _node(ptr)
  {
    _InitializeParameters();
    _UpdateParameters();

    _thread = std::thread(&LineCenterReconstruction::_Impl::_Worker, this);
  }

  ~_Impl()
  {
    _con.notify_all();
    _thread.join();
  }

  void PushBack(LineCenter::UniquePtr & ptr)
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
  }

  void _UpdateParameters()
  {
    std::vector<double> c, d, h;
    _node->get_parameter("camera_matrix", c);
    _node->get_parameter("distort_coeffs", d);
    _node->get_parameter("homography_matrix", h);

    _coef = cv::Mat(3, 3, CV_64F, c.data()).clone();
    _dist = cv::Mat(1, 5, CV_64F, d.data()).clone();
    _H = cv::Mat(3, 3, CV_64F, h.data()).clone();
  }

  void _Worker()
  {
    cv::Mat dx;

    while (rclcpp::ok()) {
      std::unique_lock<std::mutex> lk(_mutex);
      if (_deq.empty() == false) {
        auto ptr = std::move(_deq.front());
        _deq.pop_front();
        lk.unlock();
        if (ptr->header.frame_id == "-1") {
          auto pnts = std::make_unique<PointCloud2>();
          pnts->header = ptr->header;
          _node->Publish(pnts);
        } else {
        }
      } else {
        _con.wait(lk);
      }
    }
  }

private:
  LineCenterReconstruction * _node;
  cv::Mat _coef, _dist, _H;
  std::mutex _mutex;              ///< Mutex to protect shared storage
  std::condition_variable _con;   ///< Conditional variable rely on mutex
  std::deque<LineCenter::UniquePtr> _deq;
  std::thread _thread;
};

LineCenterReconstruction::LineCenterReconstruction(const rclcpp::NodeOptions & options)
: Node("line_center_reconstruction_node", options)
{
  _pub = this->create_publisher<PointCloud2>(_pubName, rclcpp::SensorDataQoS());

  _impl = std::make_unique<_Impl>(this);

  _sub = this->create_subscription<LineCenter>(
    _subName,
    rclcpp::SensorDataQoS(),
    [this](LineCenter::UniquePtr ptr)
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
  _pub.reset();

  RCLCPP_INFO(this->get_logger(), "Destroyed successfully");
}

}  // namespace line_center_reconstruction

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(line_center_reconstruction::LineCenterReconstruction)
