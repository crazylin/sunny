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

// #include <sys/socket.h>
// #include <errno.h>
#include <modbus.h>
// #include <stdio.h>
#include <unistd.h>

// #include <climits>
#include <memory>
#include <set>
// #include <string>
// #include <utility>
// #include <vector>
// #include <iostream>

namespace modbus
{

using sensor_msgs::msg::PointCloud2;

Modbus::Modbus(const rclcpp::NodeOptions & options)
: Node("modbus_node", options)
{
  _param_camera = std::make_shared<rclcpp::AsyncParametersClient>(this, "camera_tis_node");
  _param_gpio = std::make_shared<rclcpp::AsyncParametersClient>(this, "gpio_raspberry_node");

  this->declare_parameter("port", 2345);
  auto port = this->get_parameter("port").as_int();
  _thread = std::thread(&Modbus::_modbus, this, port);

  RCLCPP_INFO(this->get_logger(), "Initialized successfully");
}

Modbus::~Modbus()
{
  try {
    _thread.join();
    _param_gpio.reset();
    _param_camera.reset();
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
void Modbus::_gpio_laser(bool f)
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
void Modbus::_camera_power(bool f)
{
  if (f) {
    _param_camera->set_parameters({rclcpp::Parameter("power", true)});
  } else {
    _param_camera->set_parameters({rclcpp::Parameter("power", false)});
  }
}

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
void Modbus::_modbus(int port)
{
  auto ctx = modbus_new_tcp(NULL, port);
  if (!ctx) {
    throw std::runtime_error("Can not create modbus context");
  }

  // _mb_mapping = modbus_mapping_new(MODBUS_MAX_READ_BITS, 0, MODBUS_MAX_READ_REGISTERS, 0);
  auto mb_mapping = modbus_mapping_new(0, 0, 400, 0);
  if (!mb_mapping) {
    modbus_free(ctx);
    throw std::runtime_error("Can not initialize modbus registers");
  }
  mb_mapping->tab_registers[1] = 0xff;

  auto sock = modbus_tcp_listen(ctx, 10);
  if (sock == -1) {
    modbus_mapping_free(mb_mapping);
    modbus_free(ctx);
    throw std::runtime_error("Can not create modbus socket");
  }

  std::set<int> fds {sock};

  fd_set refset;
  FD_ZERO(&refset);
  FD_SET(sock, &refset);

  int fdmax = sock;
  uint8_t query[MODBUS_TCP_MAX_ADU_LENGTH];
  while (rclcpp::ok()) {
    auto rdset = refset;
    timeval tv;
    tv.tv_sec = 0;
    tv.tv_usec = 100000;
    if (select(fdmax + 1, &rdset, NULL, NULL, &tv) < 1) {continue;}

    auto fds_bak = fds;
    for (auto fd : fds_bak) {
      if (!FD_ISSET(fd, &rdset)) {continue;}

      if (fd == sock) {
        // A client is asking a new connection
        auto s = modbus_tcp_accept(ctx, &sock);
        if (s != -1) {
          FD_SET(s, &refset);
          fds.insert(fds.end(), s);
        }
      } else {
        // A client is asking for reply
        modbus_set_socket(ctx, fd);
        auto rc = modbus_receive(ctx, query);
        if (rc == -1) {
          // Connection closed by the client or error
          close(fd);
          FD_CLR(fd, &refset);
          fds.erase(fd);
        } else if (rc > 0) {
          if (rc > 14 && query[7] == 0x10 && query[8] == 0x01 && query[9] == 0x01) {
            if (query[14]) {
              _gpio_laser(true);
              _camera_power(true);
            } else {
              _camera_power(false);
              _gpio_laser(false);
            }
          }
          modbus_reply(ctx, query, rc, mb_mapping);
        }
      }
    }
    fdmax = *fds.rbegin();
  }

  close(sock);
  modbus_mapping_free(mb_mapping);
  modbus_free(ctx);
}

}  // namespace modbus

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(modbus::Modbus)
