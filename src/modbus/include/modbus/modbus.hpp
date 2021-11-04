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
#include "std_srvs/srv/trigger.hpp"
#include "shared_interfaces/msg/modbus_coord.hpp"

namespace modbus
{

class Modbus : public rclcpp::Node
{
public:
  explicit Modbus(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  ~Modbus();

private:
  void _Init();
  void _InitializeParameters();
  void _UpdateParameters();
  void _Sub(shared_interfaces::msg::ModbusCoord::UniquePtr ptr);

public:
  class _Impl;
  std::unique_ptr<_Impl> _impl;

  const char * _subName = "~/coord";
  rclcpp::Subscription<shared_interfaces::msg::ModbusCoord>::SharedPtr _sub;

  std::map<std::string, rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr> _map;

  std::thread _init;
};

}  // namespace modbus

#endif  // MODBUS__MODBUS_HPP_
