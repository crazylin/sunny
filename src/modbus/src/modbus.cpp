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

// #include <netinet/in.h>
// #include <sys/socket.h>
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
    RCLCPP_ERROR(this->get_logger(), "Failed to create modbus context.");
    rclcpp::shutdown();
    return;
  }

  // _mb_mapping = modbus_mapping_new(MODBUS_MAX_READ_BITS, 0, MODBUS_MAX_READ_REGISTERS, 0);
  auto mb_mapping = modbus_mapping_new(0, 0, 400, 0);
  if (!mb_mapping) {
    modbus_free(ctx);
    RCLCPP_ERROR(this->get_logger(), "Failed to initialize modbus registers.");
    rclcpp::shutdown();
    return;
  }
  mb_mapping->tab_registers[1] = 0xff;

  auto sock = modbus_tcp_listen(ctx, 10);
  if (sock == -1) {
    modbus_mapping_free(mb_mapping);
    modbus_free(ctx);
    RCLCPP_ERROR(this->get_logger(), "Failed to listen.");
    rclcpp::shutdown();
    return;
  }

  std::set<int> fds {sock};

  fd_set refset;
  FD_ZERO(&refset);
  FD_SET(sock, &refset);

  int fdmax = sock;
  uint8_t query[MODBUS_TCP_MAX_ADU_LENGTH];
  int ret = 0;
  while (rclcpp::ok() && ret != -1) {
    auto rdset = refset;
    timeval tv;
    tv.tv_sec = 0;
    tv.tv_usec = 100000;
    ret = select(fdmax + 1, &rdset, NULL, NULL, &tv);
    if (ret == -1) {
      RCLCPP_ERROR(this->get_logger(), "Failed to select.");
      continue;
    } else if (ret == 0) {
      // time out.
      continue;
    }

    auto fds_bak = fds;
    for (auto fd : fds_bak) {
      if (!FD_ISSET(fd, &rdset)) {continue;}

      if (fd == sock) {
        // A client is asking a new connection
        // struct sockaddr_in clientaddr;
        // socklen_t addrlen = sizeof(clientaddr);
        // memset(&clientaddr, 0, sizeof(clientaddr));
        // ret = accept(sock, (struct sockaddr *)&clientaddr, &addrlen);
        ret = modbus_tcp_accept(ctx, &sock);
        if (ret != -1) {
          FD_SET(ret, &refset);
          fds.insert(fds.end(), ret);
          fdmax = *fds.rbegin();
        } else {
          RCLCPP_ERROR(this->get_logger(), "Failed to accept.");
          break;
        }
      } else {
        // A client is asking for reply
        ret = modbus_set_socket(ctx, fd);
        if (ret == -1) {
          RCLCPP_ERROR(this->get_logger(), "Failed to set socket.");
          break;
        }
        ret = modbus_receive(ctx, query);
        if (ret == -1) {
          // Connection closed by the client or error
          close(fd);
          FD_CLR(fd, &refset);
          fds.erase(fd);
          fdmax = *fds.rbegin();
          ret = 0;
        } else if (ret > 0) {
          // Client request
          if (ret > 14 && query[7] == 0x10 && query[8] == 0x01 && query[9] == 0x01) {
            if (query[14]) {
              _gpio_laser(true);
              _camera_power(true);
            } else {
              _camera_power(false);
              _gpio_laser(false);
            }
          }

          ret = modbus_reply(ctx, query, ret, mb_mapping);
          // std::cout << mb_mapping->tab_registers[2] << " " << mb_mapping->tab_registers[3] << " " << mb_mapping->tab_registers[4] << "\n";
          if (ret == -1) {
            RCLCPP_ERROR(this->get_logger(), "Failed to reply.");
            break;
          }
        }
      }
    }
  }

  close(sock);
  modbus_mapping_free(mb_mapping);
  modbus_free(ctx);
  if (ret == -1) {
    rclcpp::shutdown();
  }
}

}  // namespace modbus

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(modbus::Modbus)
