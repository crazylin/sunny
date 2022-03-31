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

#include "camera_tis/camera_tis.hpp"

extern "C"
{
  #include <gst/gst.h>
  #include <gst/video/video.h>
  #include <tcamprop.h>
}

#include <exception>
#include <memory>
#include <utility>
#include <vector>

namespace camera_tis
{

using sensor_msgs::msg::Image;

const auto WIDTH = 1536, HEIGHT = 1024, SIZE = 1536 * 1024, FPS = 60;

class CameraTis::_Impl
{
public:
  explicit _Impl(CameraTis * ptr)
  : _node(ptr)
  {
    _InitializeParameters();
    _UpdateParameters();
    gst_debug_set_default_threshold(GST_LEVEL_WARNING);
    gst_init(NULL, NULL);
    const char * pipeline_str =
      "tcambin name=source "
      "! video/x-raw,format=GRAY8,width=3072,height=2048,framerate=30/1 "
      "! videoscale "
      "! video/x-raw,width=1536,height=1024 "
      "! appsink name=sink emit-signals=true sync=false drop=true max-buffers=4";
    GError * err = NULL;
    _pipeline = gst_parse_launch(pipeline_str, &err);
    if (_pipeline == NULL) {
      throw std::runtime_error("TIS pipeline fail");
    }

    _SetProperty("Exposure Auto", false);
    _SetProperty("Gain Auto", false);
    _SetProperty("Exposure Time (us)", _expo);
    _SetProperty("Brightness", 0);

    gst_element_set_state(_pipeline, GST_STATE_PAUSED);
    _thread = std::thread(&_Impl::spin, this);
  }

  ~_Impl()
  {
    gst_element_set_state(_pipeline, GST_STATE_NULL);
    _thread.join();
    gst_object_unref(_pipeline);
  }

  void spin()
  {
    GstElement * sink = gst_bin_get_by_name(GST_BIN(_pipeline), "sink");
    int frame = 0;
    while (rclcpp::ok()) {
      GstSample * sample = NULL;
      g_signal_emit_by_name(sink, "pull-sample", &sample, NULL);
      if (sample == NULL) {
        continue;
      }
      GstBuffer * buffer = gst_sample_get_buffer(sample);
      if (buffer == NULL) {
        gst_sample_unref(sample);
        continue;
      }

      GstMapInfo info;
      if (gst_buffer_map(buffer, &info, GST_MAP_READ)) {
        auto ptr = std::make_unique<Image>();
        ptr->header.stamp = _node->now();
        ptr->header.frame_id = std::to_string(frame++);
        ptr->height = HEIGHT;
        ptr->width = WIDTH;
        ptr->encoding = "mono8";
        ptr->is_bigendian = false;
        ptr->step = WIDTH;
        ptr->data.resize(SIZE);
        memcpy(ptr->data.data(), info.data, SIZE);
        _node->Publish(ptr);
        gst_buffer_unmap(buffer, &info);
      }
      gst_sample_unref(sample);
    }
    gst_object_unref(sink);
  }

  int Power(bool p)
  {
    if (p) {
      return PowerOn();
    } else {
      return PowerOff();
    }
  }

  int PowerOn()
  {
    if (!_power) {
      auto ret = gst_element_set_state(_pipeline, GST_STATE_PLAYING);
      if (ret != GST_STATE_CHANGE_FAILURE) {
        _power = true;
        return 0;
      } else {
        return 1;
      }
    } else {
      return 0;
    }
  }

  int PowerOff()
  {
    if (_power) {
      auto ret = gst_element_set_state(_pipeline, GST_STATE_PAUSED);
      if (ret != GST_STATE_CHANGE_FAILURE) {
        _power = false;
        return 0;
      } else {
        return 1;
      }
    } else {
      return 0;
    }
  }

  void _InitializeParameters()
  {
    _node->declare_parameter("exposure_time", _expo);
    _node->declare_parameter("power", _power);
  }

  void _UpdateParameters()
  {
    _node->get_parameter("exposure_time", _expo);
    _node->get_parameter("power", _power);
  }

  gboolean _SetProperty(const char * property, const char * value)
  {
    gboolean ret = FALSE;
    GstElement * bin = gst_bin_get_by_name(GST_BIN(_pipeline), "source");
    GValue val = G_VALUE_INIT;
    g_value_init(&val, G_TYPE_STRING);
    g_value_set_string(&val, value);
    ret = tcam_prop_set_tcam_property(TCAM_PROP(bin), property, &val);
    g_value_unset(&val);
    gst_object_unref(bin);
    return ret;
  }

  gboolean _SetProperty(const char * property, int value)
  {
    gboolean ret = FALSE;
    GstElement * bin = gst_bin_get_by_name(GST_BIN(_pipeline), "source");
    GValue type = {};
    tcam_prop_get_tcam_property(
      TCAM_PROP(bin),
      property,
      NULL,
      NULL, NULL, NULL, NULL,
      &type, NULL, NULL, NULL);
    const char * t = g_value_get_string(&type);
    if (strcmp(t, "boolean") == 0) {
      GValue val = G_VALUE_INIT;
      g_value_init(&val, G_TYPE_BOOLEAN);
      g_value_set_boolean(&val, value);
      ret = tcam_prop_set_tcam_property(TCAM_PROP(bin), property, &val);
      g_value_unset(&val);
    } else {
      GValue val = G_VALUE_INIT;
      g_value_init(&val, G_TYPE_INT);
      g_value_set_int(&val, value);
      ret = tcam_prop_set_tcam_property(TCAM_PROP(bin), property, &val);
      g_value_unset(&val);
    }
    g_value_unset(&type);
    gst_object_unref(bin);
    return ret;
  }

private:
  CameraTis * _node;

  GstElement * _pipeline;
  int _expo = 1000;
  bool _power = false;

  std::thread _thread;
};

CameraTis::CameraTis(const rclcpp::NodeOptions & options)
: Node("camera_tis_node", options)
{
  _init = std::thread(&CameraTis::_Init, this);
}

CameraTis::~CameraTis()
{
  try {
    _init.join();
    _impl.reset();
    _pubImage.reset();

    RCLCPP_INFO(this->get_logger(), "Destroyed successfully");
  } catch (const std::exception & e) {
    RCLCPP_ERROR(this->get_logger(), "Exception in destructor: %s", e.what());
  } catch (...) {
    RCLCPP_ERROR(this->get_logger(), "Exception in destructor: unknown");
  }
}

void CameraTis::_Init()
{
  try {
    _pubImage = this->create_publisher<Image>(_pubImageName, rclcpp::SensorDataQoS());

    _impl = std::make_unique<_Impl>(this);

    _parCallbackHandle = this->add_on_set_parameters_callback(
      [this](const std::vector<rclcpp::Parameter> & parameters) {
        rcl_interfaces::msg::SetParametersResult result;
        result.successful = true;
        for (const auto & parameter : parameters) {
          if (parameter.get_name() == "exposure_time") {
            auto ret = this->_impl->_SetProperty("Exposure Time (us)", parameter.as_int());
            if (ret != TRUE) {
              result.successful = false;
              result.reason = "Failed to set exposure time";
            }
          } else if (parameter.get_name() == "power") {
            auto ret = this->_impl->Power(parameter.as_bool());
            if (ret) {
              result.successful = false;
              result.reason = "Failed to set power";
            }
          }
        }
        return result;
      });
    RCLCPP_INFO(this->get_logger(), "Initialized successfully");
  } catch (const std::exception & e) {
    RCLCPP_ERROR(this->get_logger(), "Exception in initializer: %s", e.what());
    rclcpp::shutdown();
  } catch (...) {
    RCLCPP_ERROR(this->get_logger(), "Exception in initializer: unknown");
    rclcpp::shutdown();
  }
}

}  // namespace camera_tis

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(camera_tis::CameraTis)
