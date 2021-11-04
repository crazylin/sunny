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
    #include <unistd.h>
}

#include <climits>
#include <memory>
#include <string>
#include <utility>
#include <vector>

namespace modbus
{

using namespace std::chrono_literals;
using std_srvs::srv::Trigger;
using shared_interfaces::msg::ModbusCoord;

bool EndsWith(const std::string & value, const std::string & ending)
{
  if (value.size() < ending.size()) {
    return false;
  }
  return std::equal(ending.rbegin(), ending.rend(), value.rbegin());
}

void CheckAndSend(rclcpp::Client<Trigger>::SharedPtr ptr)
{
  auto t = std::make_shared<Trigger::Request>();

  if (rclcpp::ok() && ptr->service_is_ready()) {
    ptr->async_send_request(t);
  }
}

class Modbus::_Impl
{
public:
  explicit _Impl(Modbus * ptr)
  : _node(ptr)
  {
    ctx = modbus_new_tcp(NULL, 502);
    if (!ctx) {
      throw std::runtime_error("Can not create modbus context");
    }

    header_length = modbus_get_header_length(ctx);

    mb_mapping = modbus_mapping_new(0, 0, 40, 0);
    if (!mb_mapping) {
      modbus_free(ctx);
      throw std::runtime_error("Can not initialize modbus registers");
    }

    std::thread temp(
      [this]() {
        while (rclcpp::ok()) {
          _ListenAndAccept();
          _Receive();
        }
      }
    );

    temp.detach();
  }

  ~_Impl()
  {
    close(sock);
    modbus_mapping_free(mb_mapping);
    modbus_close(ctx);
    modbus_free(ctx);
  }

  void Update(bool valid, double x = 0, double y = 0, double z = 0)
  {
    std::lock_guard<std::mutex> lock(_mutex);
    if (z * 10000 > SHRT_MAX) {
      mb_mapping->tab_registers[16] = 0;
      mb_mapping->tab_registers[17] = 0;
      mb_mapping->tab_registers[18] = 0;
      mb_mapping->tab_registers[19] = 0;
    } else {
      mb_mapping->tab_registers[16] = (uint16_t) valid;
      mb_mapping->tab_registers[17] = (uint16_t) x;
      mb_mapping->tab_registers[18] = (uint16_t) y * 10000;
      mb_mapping->tab_registers[19] = (uint16_t) z * 10000;
    }
  }

private:
  void _ListenAndAccept()
  {
    while (rclcpp::ok()) {
      close(sock);
      sock = modbus_tcp_listen(ctx, 1);
      if (sock != -1 && modbus_tcp_accept(ctx, &sock) != -1) {
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Initialized listen and accept successfully");
        return;
      }

      std::this_thread::sleep_for(200ms);
    }
  }

  void _Receive()
  {
    while (rclcpp::ok()) {
      int rc;
      do {
        rc = modbus_receive(ctx, query);
      } while (rc == 0 && rclcpp::ok());

      if (rc <= 0) {break;}

      if (query[7] == 0x06 && query[9] == 0x01) {
        if (query[11]) {
          CheckAndSend(_node->_map["/camera_spinnaker_node/start"]);
        } else {
          CheckAndSend(_node->_map["/camera_spinnaker_node/stop"]);
        }
      }
      std::lock_guard<std::mutex> lock(_mutex);
      modbus_reply(ctx, query, rc, mb_mapping);
    }
  }

private:
  Modbus * _node;
  modbus_t * ctx = NULL;
  int header_length;
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
    _sub.reset();
    _impl.reset();
    RCLCPP_INFO(this->get_logger(), "Destroyed successfully");
  } catch (const std::exception & e) {
    RCLCPP_FATAL(this->get_logger(), "Exception in destructor: %s", e.what());
  } catch (...) {
    RCLCPP_FATAL(this->get_logger(), "Exception in destructor: unknown");
  }
}

void Modbus::_Init()
{
  _InitializeParameters();

  _UpdateParameters();

  while (rclcpp::ok()) {
    auto srvs = this->get_service_names_and_types();
    auto pos = std::find_if(
      srvs.begin(), srvs.end(), [](const std::pair<std::string, std::vector<std::string>> & p) {
        return EndsWith(p.first, "/camera_spinnaker_node/start");
      }
    );

    if (pos == srvs.end()) {
      std::this_thread::sleep_for(200ms);
      continue;
    } else {
      _map[pos->first] = nullptr;
      break;
    }
  }

  while (rclcpp::ok()) {
    auto srvs = this->get_service_names_and_types();
    auto pos = std::find_if(
      srvs.begin(), srvs.end(), [](const std::pair<std::string, std::vector<std::string>> & p) {
        return EndsWith(p.first, "/camera_spinnaker_node/stop");
      }
    );

    if (pos == srvs.end()) {
      std::this_thread::sleep_for(200ms);
      continue;
    } else {
      _map[pos->first] = nullptr;
      break;
    }
  }

  for (auto & p : _map) {
    const auto & n = p.first;
    p.second = this->create_client<Trigger>(n);
    RCLCPP_DEBUG(rclcpp::get_logger("rclcpp"), "Services ready [%s]", n.c_str());
  }

  _impl = std::make_unique<_Impl>(this);

  _sub = this->create_subscription<ModbusCoord>(
    _subName, 10, std::bind(&Modbus::_Sub, this, std::placeholders::_1));

  RCLCPP_INFO(this->get_logger(), "Initialized successfully");
}

void Modbus::_Sub(ModbusCoord::UniquePtr ptr)
{
  _impl->Update(ptr->a, ptr->b, ptr->c, ptr->d);
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
