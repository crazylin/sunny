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

#include "modbus/modbus.hpp"

#include <errno.h>
#include <modbus.h>
#include <stdio.h>
#include <unistd.h>

#include <climits>
#include <memory>
#include <string>
#include <utility>
#include <vector>
#include <iostream>

namespace modbus
{

using sensor_msgs::msg::PointCloud2;

/**
 * @brief Inner implementation for the algorithm.
 *
 */
class Modbus::_Impl
{
public:
  /**
   * @brief Construct a new impl object.
   *
   * Declare parameters before usage.
   * Establish a new TCP modbus via a specific port.
   * Listen to port.
   * Initialize a mapping block.
   * Start a thread to recursively accept and receive requests.
   * @param ptr Reference to parent node.
   */
  explicit _Impl(Modbus * ptr)
  : _node(ptr)
  {
    _node->declare_parameter("port", 2345);
    auto port = _node->get_parameter("port").as_int();

    _ctx = modbus_new_tcp(NULL, port);
    if (!_ctx) {
      throw std::runtime_error("Can not create modbus context");
    }

    _sock = modbus_tcp_listen(_ctx, 1);
    if (_sock == -1) {
      modbus_free(_ctx);
      throw std::runtime_error("Can not create modbus socket");
    }

    _mb_mapping = modbus_mapping_new(0, 0, 400, 0);
    if (!_mb_mapping) {
      close(_sock);
      modbus_free(_ctx);
      throw std::runtime_error("Can not initialize modbus registers");
    }

    _mb_mapping->tab_registers[1] = 0xff;
    std::thread(
      [this]() {
        while (rclcpp::ok()) {
          if (modbus_tcp_accept(_ctx, &_sock) == -1) {
            RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "modbus tcp accept error");
            rclcpp::shutdown();
            break;
          }
          receive();
        }
      }).detach();
  }

  /**
   * @brief Destroy the impl object.
   *
   * Close the socket if it is open.
   * Free mapping block.
   * Free modbus context.
   */
  ~_Impl()
  {
    if (_sock != -1) {
      close(_sock);
    }
    modbus_mapping_free(_mb_mapping);
    modbus_free(_ctx);
  }

  /**
   * @brief Update coordinates in mapping block.
   *
   * @param valid true if found.
   * @param u Coordinates in u.
   * @param v Coordinates in v.
   */
  void update(bool valid, float u = 0., float v = 0.)
  {
    std::lock_guard<std::mutex> lock(_mutex);

    if (valid) {
      _mb_mapping->tab_registers[2] = 0xff;
      _mb_mapping->tab_registers[3] = static_cast<uint16_t>(u * 100);
      _mb_mapping->tab_registers[4] = static_cast<uint16_t>(v * 100);
    } else {
      _mb_mapping->tab_registers[2] = 0;
    }
  }

  /**
   * @brief Receive and reply data request from master.
   *
   */
  void receive()
  {
    while (rclcpp::ok()) {
      uint8_t query[MODBUS_TCP_MAX_ADU_LENGTH];
      int rc = modbus_receive(_ctx, query);
      if (rc == -1) {
        // Connection closed by the client or error
        break;
      } else if (rc == 0) {
        continue;
      }

      // Received request
      if (rc > 14 && query[7] == 0x10 && query[8] == 0x01 && query[9] == 0x01) {
        if (query[14]) {
          _node->gpio_laser(true);
          _node->camera_power(true);
        } else {
          _node->camera_power(false);
          _node->gpio_laser(false);
        }
      }
      std::lock_guard<std::mutex> lock(_mutex);
      modbus_reply(_ctx, query, rc, _mb_mapping);
    }
  }

private:
  Modbus * _node;
  modbus_t * _ctx = NULL;
  modbus_mapping_t * _mb_mapping = NULL;
  int _sock = -1;
  std::mutex _mutex;
};

/**
 * @brief Construct a new Modbus object.
 *
 * Create an inner implementation.
 * Initialize subscription.
 * Initialize parameter client for camera.
 * Initialize parameter client for gpio.
 * Print success if all done.
 * @param options Encapsulation of options for node initialization.
 */
Modbus::Modbus(const rclcpp::NodeOptions & options)
: Node("modbus_node", options)
{
  _impl = std::make_unique<_Impl>(this);

  _sub = this->create_subscription<PointCloud2>(
    _sub_name,
    rclcpp::SensorDataQoS(),
    [this](PointCloud2::UniquePtr ptr) {
      if (ptr->data.empty()) {
        _impl->update(false, 0., 0.);
      } else {
        float * p = reinterpret_cast<float *>(ptr->data.data());
        if (p[2] == -1) {
          _impl->update(true, p[0], p[1]);
        } else {
          _impl->update(false, 0., 0.);
        }
      }
    });

  _param_camera = std::make_shared<rclcpp::AsyncParametersClient>(this, "camera_tis_node");
  _param_gpio = std::make_shared<rclcpp::AsyncParametersClient>(this, "gpio_raspberry_node");

  RCLCPP_INFO(this->get_logger(), "Initialized successfully");
}

/**
 * @brief Destroy the Modbus:: Modbus object.
 *
 * Release parameter client for camera.
 * Release parameter client for gpio.
 * Release subscription.
 * Release inner implementation.
 * Print success if all done.
 * Throw no exception.
 */
Modbus::~Modbus()
{
  try {
    _param_gpio.reset();
    _param_camera.reset();
    _sub.reset();
    _impl.reset();
    RCLCPP_INFO(this->get_logger(), "Destroyed successfully");
  } catch (const std::exception & e) {
    RCLCPP_FATAL(this->get_logger(), "Exception in destructor: %s", e.what());
  } catch (...) {
    RCLCPP_FATAL(this->get_logger(), "Exception in destructor: unknown");
  }
}

/**
 * @brief Control laser on of off.
 *
 * @param f true if on.
 */
void Modbus::gpio_laser(bool f)
{
  if (f) {
    _param_gpio->set_parameters({rclcpp::Parameter("laser", true)});
  } else {
    _param_gpio->set_parameters({rclcpp::Parameter("laser", false)});
  }
}

/**
 * @brief Control camera capture or not.
 *
 * @param f true if enable camera capture.
 */
void Modbus::camera_power(bool f)
{
  if (f) {
    _param_camera->set_parameters({rclcpp::Parameter("power", true)});
  } else {
    _param_camera->set_parameters({rclcpp::Parameter("power", false)});
  }
}

}  // namespace modbus

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(modbus::Modbus)
