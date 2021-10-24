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

#include "camera_mvs/camera_mvs.hpp"

#include <exception>
#include <memory>

extern "C"
{
    #include "MvCameraControl.h"
}

namespace camera_mvs
{

using namespace std::chrono_literals;
using sensor_msgs::msg::Image;
using std_srvs::srv::Trigger;

void __stdcall ImageCallBackEx(
  unsigned char * pData,
  MV_FRAME_OUT_INFO_EX * pFrameInfo,
  void * pUser)
{
  try {
    if (pFrameInfo) {
      auto node = static_cast<CameraMvs *>(pUser);
      auto ptr = std::make_unique<Image>();
      ptr->header.stamp = node->now();
      ptr->header.frame_id = std::to_string(pFrameInfo->nFrameNum);
      ptr->height = pFrameInfo->nHeight;
      ptr->width = pFrameInfo->nWidth;
      ptr->encoding = "mono8";
      ptr->is_bigendian = false;
      ptr->step = pFrameInfo->nWidth;
      ptr->data.resize(pFrameInfo->nFrameLen);
      memcpy(ptr->data.data(), pData, pFrameInfo->nFrameLen);

      node->Publish(ptr);
    }
  } catch (const std::exception & e) {
    RCLCPP_WARN(rclcpp::get_logger("rclcpp"), "Exception in camera callback: %s", e.what());
  } catch (...) {
    RCLCPP_WARN(rclcpp::get_logger("rclcpp"), "Exception in camera callback: unknown");
  }
}

class CameraMvs::_Impl
{
public:
  explicit _Impl(CameraMvs * ptr)
  : _node(ptr)
  {
    _InitializeParameters();

    while (true) {
      if (!rclcpp::ok()) {
        throw std::runtime_error("Interrupted!");
      }

      memset(&stDeviceList, 0, sizeof(MV_CC_DEVICE_INFO_LIST));

      if (MV_OK != MV_CC_EnumDevices(MV_USB_DEVICE, &stDeviceList)) {
        throw std::runtime_error("Can not enum Hik mvs devices!");
      } else if (stDeviceList.nDeviceNum > 0) {
        break;
      }

      std::this_thread::sleep_for(2000ms);
    }

    if (MV_OK != MV_CC_CreateHandle(&_handle, stDeviceList.pDeviceInfo[0])) {
      throw std::runtime_error("MV_CC_CreateHandle fail");
    }

    if (MV_OK != MV_CC_OpenDevice(_handle)) {
      throw std::runtime_error("MV_CC_OpenDevice fail!");
    }

    if (MV_OK != MV_CC_SetEnumValue(_handle, "TriggerMode", 0)) {
      throw std::runtime_error("MV_CC_SetTriggerMode fail!");
    }

    if (MV_OK != MV_CC_RegisterImageCallBackEx(_handle, ImageCallBackEx, _node)) {
      throw std::runtime_error("MV_CC_RegisterImageCallBackEx fail!");
    }

    _WarmUp();
  }

  ~_Impl()
  {
    MV_CC_StopGrabbing(_handle);
    MV_CC_CloseDevice(_handle);
    MV_CC_DestroyHandle(_handle);
  }

  void Start()
  {
    _UpdateParameters();

    _ApplyParameters();

    _StreamOn();
  }

  void Stop()
  {
    _StreamOff();
  }

private:
  void _InitializeParameters()
  {
    /*
    _node->declare_parameter("acqusition_buffer_number", _acqusitionBufferNumber);
    _node->declare_parameter("acqusition_frame_rate", _acqusitionFrameRate);
    _node->declare_parameter("exposure_time", _exposureTime);
    */
  }

  void _UpdateParameters()
  {
    /*
    _node->get_parameter("acqusition_buffer_number", _acqusitionBufferNumber);
    _node->get_parameter("acqusition_frame_rate", _acqusitionFrameRate);
    _node->get_parameter("exposure_time", _exposureTime);
    */
  }

  void _ApplyParameters()
  {
    /*
    if (GX_STATUS_SUCCESS !=
      GXSetAcqusitionBufferNumber(_handle, _acqusitionBufferNumber))
    {
      throw std::runtime_error("Can not set paramter: acqusition buffer number");
    }

    if (GX_STATUS_SUCCESS !=
      GXSetFloat(_handle, GX_FLOAT_ACQUISITION_FRAME_RATE, _acqusitionFrameRate))
    {
      throw std::runtime_error("Can not set paramter: acqusition frame rate");
    }

    if (GX_STATUS_SUCCESS !=
      GXSetFloat(_handle, GX_FLOAT_EXPOSURE_TIME, _exposureTime))
    {
      throw std::runtime_error("Can not set paramter: exposure time");
    }
    */
  }

  void _StreamOn()
  {
    if (MV_OK != MV_CC_StartGrabbing(_handle)) {
      throw std::runtime_error("Error: stream on");
    }
  }

  void _StreamOff()
  {
    if (MV_OK != MV_CC_StopGrabbing(_handle)) {
      throw std::runtime_error("Error: stream off");
    }
  }

  void _WarmUp()
  {
    /*
    _UpdateParameters();

    _ApplyParameters();

    _StreamOn();

    for (int i = 0; i < 25; ++i) {
      if (!rclcpp::ok()) {
        GXStreamOff(_handle);
        GXCloseLib();
        throw std::runtime_error("Interrupted!");
      }
      std::this_thread::sleep_for(200ms);
    }

    _StreamOff();
    */
  }

private:
  CameraMvs * _node;
  void * _handle = NULL;
  MV_CC_DEVICE_INFO_LIST stDeviceList;
  int _acqusitionBufferNumber = 300;    ///< Acqusition buffer number
  double _acqusitionFrameRate = 60.;    ///< Acqusition frame rate
  double _exposureTime = 200.;          ///< Exposure time: us
};

CameraMvs::CameraMvs(const rclcpp::NodeOptions & options)
: Node("camera_mvs_node", options)
{
  _init = std::thread(&CameraMvs::_Init, this);
}

CameraMvs::~CameraMvs()
{
  try {
    _init.join();

    _srvStart.reset();
    _srvStop.reset();
    _impl.reset();
    _pub.reset();

    RCLCPP_INFO(this->get_logger(), "Destroyed successfully");
  } catch (const std::exception & e) {
    RCLCPP_ERROR(this->get_logger(), "Exception in destructor: %s", e.what());
  } catch (...) {
    RCLCPP_ERROR(this->get_logger(), "Exception in destructor: unknown");
  }
}

void CameraMvs::_Init()
{
  try {
    _pub = this->create_publisher<Image>(_pubName, rclcpp::SensorDataQoS());

    _impl = std::make_unique<_Impl>(this);

    _srvStop = this->create_service<Trigger>(
      _srvStopName,
      std::bind(&CameraMvs::_Stop, this, std::placeholders::_1, std::placeholders::_2));

    _srvStart = this->create_service<Trigger>(
      _srvStartName,
      std::bind(&CameraMvs::_Start, this, std::placeholders::_1, std::placeholders::_2));

    RCLCPP_INFO(this->get_logger(), "Initialized successfully");
  } catch (const std::exception & e) {
    RCLCPP_ERROR(this->get_logger(), "Exception in initializer: %s", e.what());
    rclcpp::shutdown();
  } catch (...) {
    RCLCPP_ERROR(this->get_logger(), "Exception in initializer: unknown");
    rclcpp::shutdown();
  }
}

void CameraMvs::_Start(
  const std::shared_ptr<Trigger::Request>,
  std::shared_ptr<Trigger::Response> response)
{
  try {
    response->success = false;
    response->message = "Fail: camera start";

    _impl->Start();

    response->success = true;
    response->message = "Success: camera start";
  } catch (const std::exception & e) {
    RCLCPP_WARN(this->get_logger(), "Exception in service start: %s", e.what());
  } catch (...) {
    RCLCPP_WARN(this->get_logger(), "Exception in service start: unknown");
  }
}

void CameraMvs::_Stop(
  const std::shared_ptr<Trigger::Request>,
  std::shared_ptr<Trigger::Response> response)
{
  try {
    response->success = false;
    response->message = "Fail: camera stop";

    _impl->Stop();

    response->success = true;
    response->message = "Success: camera stop";
  } catch (const std::exception & e) {
    RCLCPP_ERROR(this->get_logger(), "Exception in service stop: %s", e.what());
    rclcpp::shutdown();
  } catch (...) {
    RCLCPP_ERROR(this->get_logger(), "Exception in service stop: unknown");
    rclcpp::shutdown();
  }
}

}  // namespace camera_mvs

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(camera_mvs::CameraMvs)
