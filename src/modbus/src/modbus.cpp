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

extern "C"
{
    #include <errno.h>
    #include <modbus.h>
    #include <stdio.h>
    #include <unistd.h>
}

#include <climits>
#include <memory>
#include <string>
#include <utility>
#include <vector>
#include <iostream>

namespace modbus
{

using namespace std::chrono_literals;
using sensor_msgs::msg::PointCloud2;

/*
void AsynSendRequest(
  rclcpp::Client<SetParameters>::SharedPtr cli,
  const char* name,
  bool value)
{
  if (cli->service_is_ready()) {
    auto r = std::make_shared<SetParameters::Request::SharedPtr>();
    r->parameters.push_back(rclcpp::Parameter(name, value));
    cli->async_send_request(r);
  }
}
*/

class Modbus::_Impl
{
public:
  explicit _Impl(Modbus * ptr)
  : _node(ptr)
  {
    ctx = modbus_new_tcp(NULL, 2345);
    if (!ctx) {
      throw std::runtime_error("Can not create modbus context");
    }

    mb_mapping = modbus_mapping_new(0, 0, 400, 0);
    if (!mb_mapping) {
      modbus_free(ctx);
      throw std::runtime_error("Can not initialize modbus registers");
    }

    mb_mapping->tab_registers[1] = 255;
    std::thread _thread(
      [this]() {
        while (rclcpp::ok()) {
          _ListenAndAccept();
          _Receive();
          RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Initialized listen and accept stopped");
        }
      }
    );

    _thread.detach();
  }

  ~_Impl()
  {
    if (sock != -1) {
      close(sock);
    }
    modbus_mapping_free(mb_mapping);
    modbus_close(ctx);
    modbus_free(ctx);
  }

  void Update(bool valid, float /*x = 0.*/, float y = 0., float z = 0.)
  {
    std::lock_guard<std::mutex> lock(_mutex);

    if (valid) {
      mb_mapping->tab_registers[2] = 255;
      mb_mapping->tab_registers[3] = static_cast<uint16_t>(y * 100 + 2000);
      mb_mapping->tab_registers[4] = static_cast<uint16_t>(z * 100 + 2000);
    } else {
      mb_mapping->tab_registers[2] = 0;
    }
  }

private:
  void _ListenAndAccept()
  {
    if (sock != -1) {
      close(sock);
    }
    sock = modbus_tcp_listen(ctx, 1);
    if (sock != -1 && modbus_tcp_accept(ctx, &sock) != -1) {
      RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Initialized listen and accept successfully");
    } else {
      RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Initialized listen and accept failed");
    }
  }

  void _Receive()
  {
    while (rclcpp::ok()) {
      int rc;
      do {
        rc = modbus_receive(ctx, query);
      } while (rc == 0 && rclcpp::ok());

      if (rc <= 0) {
        break;
      }

      // for (int i = 0; i < rc; ++i) {
      // //std::cout << i << ' ' << int(query[i]) << std::endl;
      //   printf("%02d: %02hhx\n", i, query[i]);
      // }
      // std::cout << "========================\n" << std::endl;

      if (query[7] == 0x10 && query[8] == 0x01 && query[9] == 0x01) {
        if (query[14]) {
          _node->GpioLaser(true);
          _node->CameraPower(true);
        } else {
          _node->CameraPower(false);
          _node->GpioLaser(false);
        }
      }

      std::lock_guard<std::mutex> lock(_mutex);
      modbus_reply(ctx, query, rc, mb_mapping);
    }
  }

private:
  Modbus * _node;
  modbus_t * ctx = NULL;
  modbus_mapping_t * mb_mapping = NULL;
  int sock = -1;
  unsigned char query[MODBUS_TCP_MAX_ADU_LENGTH];
  std::mutex _mutex;
};

Modbus::Modbus(const rclcpp::NodeOptions & options)
: Node("modbus_node", options)
{
  _init = std::thread(&Modbus::_Init, this);
}

Modbus::~Modbus()
{
  try {
    _init.join();
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

void Modbus::GpioLaser(bool f)
{
  if (f) {
    _param_gpio->set_parameters({rclcpp::Parameter("laser", true)});
  } else {
    _param_gpio->set_parameters({rclcpp::Parameter("laser", false)});
  }
}

void Modbus::CameraPower(bool f)
{
  if (f) {
    _param_camera->set_parameters({rclcpp::Parameter("power", true)});
  } else {
    _param_camera->set_parameters({rclcpp::Parameter("power", false)});
  }
}

void Modbus::_Init()
{
  _InitializeParameters();

  _UpdateParameters();

  _impl = std::make_unique<_Impl>(this);

  _sub = this->create_subscription<PointCloud2>(
    _subName,
    rclcpp::SensorDataQoS(),
    std::bind(&Modbus::_Sub, this, std::placeholders::_1));

  _param_camera = std::make_shared<rclcpp::AsyncParametersClient>(this, "camera_tis_node");
  _param_gpio = std::make_shared<rclcpp::AsyncParametersClient>(this, "gpio_raspberry_node");

  RCLCPP_INFO(this->get_logger(), "Initialized successfully");
}

void Modbus::_Sub(PointCloud2::UniquePtr ptr)
{
  if (ptr->height == 0 || ptr->width == 0) {
    _impl->Update(false, 0., 0., 0.);
  } else {
    float * p = reinterpret_cast<float *>(ptr->data.data());
    _impl->Update(true, 0., p[0], p[1]);
  }
}

void Modbus::_InitializeParameters()
{
  // this->declare_parameter("");
}

void Modbus::_UpdateParameters()
{
  // this->get_parameter("", );
}

}  // namespace modbus

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(modbus::Modbus)
