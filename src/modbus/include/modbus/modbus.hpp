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

#ifndef MODBUS__MODBUS_HPP_
#define MODBUS__MODBUS_HPP_

#include <map>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"

namespace modbus
{

class Modbus : public rclcpp::Node
{
public:
  explicit Modbus(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  ~Modbus();

public:
  void GpioLaser(bool);
  void CameraPower(bool);

private:
  void _Init();
  void _InitializeParameters();
  void _UpdateParameters();
  void _Sub(sensor_msgs::msg::PointCloud2::UniquePtr ptr);

private:
  class _Impl;
  std::unique_ptr<_Impl> _impl;

  const char * _subName = "~/seam";
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr _sub;

  std::shared_ptr<rclcpp::AsyncParametersClient> _param_camera;
  std::shared_ptr<rclcpp::AsyncParametersClient> _param_gpio;

  std::thread _init;
};

}  // namespace modbus

#endif  // MODBUS__MODBUS_HPP_
