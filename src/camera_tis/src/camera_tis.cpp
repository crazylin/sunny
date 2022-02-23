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

namespace camera_tis
{

using std_srvs::srv::Trigger;
using sensor_msgs::msg::Image;

const auto WIDTH = 1440, HEIGHT = 1080, FPS = 60, EXPO = 1000;
rcl_interfaces::msg::SetParametersResult
param_callback(const std::vector<rclcpp::Parameter> & parameters)
{
  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;
  for (const auto & parameter : parameters) {
    if (parameter.get_name() == "exposure_time") {
      
      result.successful = false;
      result.reason = "the reason it could not be allowed";
    }
  }
  return result;
}

/*
  This function will be called in a separate thread when our appsink
  says there is data for us. user_data has to be defined
  when calling g_signal_connect. It can be used to pass objects etc.
  from your other function to the callback.
*/
extern "C" GstFlowReturn callback(GstElement * sink, void * user_data)
{
  auto node = static_cast<CameraTis *>(user_data);
  GstSample * sample = NULL;
  /* Retrieve the buffer */
  g_signal_emit_by_name(sink, "pull-sample", &sample, NULL);
  if (sample) {
    static guint framecount = 0;
    GstBuffer * buffer = gst_sample_get_buffer(sample);
    GstMapInfo info;  // contains the actual image
    if (gst_buffer_map(buffer, &info, GST_MAP_READ)) {
      GstVideoInfo * video_info = gst_video_info_new();
      if (!gst_video_info_from_caps(video_info, gst_sample_get_caps(sample))) {
        RCLCPP_WARN(rclcpp::get_logger("rclcpp"), "Failed to parse video info");
        rclcpp::shutdown();
        return GST_FLOW_ERROR;
      }
      /* Get a pointer to the image data */
      unsigned char * data = info.data;
      auto ptr = std::make_unique<Image>();
      ptr->header.stamp = node->now();
      ptr->header.frame_id = std::to_string(framecount);
      ptr->height = HEIGHT;
      ptr->width = WIDTH;
      ptr->encoding = "mono8";
      ptr->is_bigendian = false;
      ptr->step = WIDTH;
      ptr->data.resize(HEIGHT * WIDTH);
      memcpy(ptr->data.data(), data, HEIGHT * WIDTH);
      node->Publish(ptr);

      gst_buffer_unmap(buffer, &info);
      gst_video_info_free(video_info);

      // time_t T = time(NULL);
      // struct tm tm = *localtime(&T);
      // printf("f:%d, t:%d\n", framecount, tm.tm_sec);
    }
    framecount++;
    gst_sample_unref(sample);
    return GST_FLOW_OK;
  } else {
    return GST_FLOW_ERROR;
  }
}

gboolean block_until_playing(GstElement * pipeline)
{
  while (TRUE) {
    GstState state;
    GstState pending;

    // wait 0.5 seconds for something to happen
    GstStateChangeReturn ret = gst_element_get_state(pipeline, &state, &pending, 500000000);

    if (ret == GST_STATE_CHANGE_SUCCESS) {
      return TRUE;
    } else if (ret == GST_STATE_CHANGE_FAILURE) {
      return FALSE;
    }
  }
}

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
      "! capsfilter name=filter "
      "! videoconvert "
      "! appsink name=sink emit-signals=true sync=false drop=true max-buffers=4";
    GError * err = NULL;
    _pipeline = gst_parse_launch(pipeline_str, &err);
    if (_pipeline == NULL) {
      throw std::runtime_error("TIS pipeline fail");
    }

    _SetProperty("Exposure Auto", FALSE);
    _SetProperty("Gain Auto", FALSE);
    _SetProperty("Exposure", EXPO);

    _SetCaps("GRAY8", WIDTH, HEIGHT, FPS);

    /* retrieve the appsink from the pipeline */
    GstElement * sink = gst_bin_get_by_name(GST_BIN(_pipeline), "sink");

    // tell appsink what function to call when it notifies us
    g_signal_connect(sink, "new-sample", G_CALLBACK(callback), _node);

    gst_object_unref(sink);

    block_until_playing(_pipeline);
  }

  ~_Impl()
  {
    gst_element_set_state(_pipeline, GST_STATE_NULL);
    gst_object_unref(_pipeline);
  }

  void Start()
  {
    gst_element_set_state(_pipeline, GST_STATE_PLAYING);
  }

  void Stop()
  {
    gst_element_set_state(_pipeline, GST_STATE_PAUSED);
  }

  void _InitializeParameters()
  {
    _node->declare_parameter("exposure_time", _expo);
  }

  void _UpdateParameters()
  {
    _node->get_parameter("exposure_time", _expo);
  }

  gboolean _SetProperty(const char * property, bool value)
  {
    GstElement * bin = gst_bin_get_by_name(GST_BIN(_pipeline), "source");
    GValue val = G_VALUE_INIT;
    g_value_init(&val, G_TYPE_BOOLEAN);
    g_value_set_boolean(&val, value);
    gboolean ret = tcam_prop_set_tcam_property(TCAM_PROP(bin), property, &val);
    g_value_unset(&val);
    gst_object_unref(bin);
    return ret;
  }

  gboolean _SetProperty(const char * property, int value)
  {
    GstElement * bin = gst_bin_get_by_name(GST_BIN(_pipeline), "source");
    GValue val = G_VALUE_INIT;
    g_value_init(&val, G_TYPE_INT);
    g_value_set_int(&val, value);
    gboolean ret = tcam_prop_set_tcam_property(TCAM_PROP(bin), property, &val);
    g_value_unset(&val);
    gst_object_unref(bin);
    return ret;
  }

  gboolean _SetCaps(const char * format, int width, int height, int fps)
  {
    GstElement * bin = gst_bin_get_by_name(GST_BIN(_pipeline), "filter");

    GstCaps * caps = gst_caps_new_empty();

    GstStructure * structure = gst_structure_from_string("video/x-raw", NULL);
    gst_structure_set(
      structure,
      "format", G_TYPE_STRING, format,
      "width", G_TYPE_INT, width,
      "height", G_TYPE_INT, height,
      "framerate", GST_TYPE_FRACTION, fps, 1,
      NULL);

    gst_caps_append_structure(caps, structure);

    g_object_set(G_OBJECT(bin), "caps", caps, NULL);
    gst_caps_unref(caps);
    gst_object_unref(bin);
    return TRUE;
  }

private:
  CameraTis * _node;
  GstElement * _pipeline;
  int _expo = 1000;
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

void CameraTis::_Init()
{
  try {
    _pub = this->create_publisher<Image>(_pubName, rclcpp::SensorDataQoS());

    _impl = std::make_unique<_Impl>(this);

    _srvStop = this->create_service<Trigger>(
      _srvStopName,
      std::bind(&CameraTis::_Stop, this, std::placeholders::_1, std::placeholders::_2));

    _srvStart = this->create_service<Trigger>(
      _srvStartName,
      std::bind(&CameraTis::_Start, this, std::placeholders::_1, std::placeholders::_2));

    _parCallbackHandle = this->add_on_set_parameters_callback(
      [this] (const std::vector<rclcpp::Parameter> & parameters) {
        rcl_interfaces::msg::SetParametersResult result;
        result.successful = true;
        for (const auto & parameter : parameters) {
          if (parameter.get_name() == "exposure_time") {
            auto ret = this->_impl->_SetProperty("Exposure", parameter.as_int());
            if (ret != TRUE) { 
              result.successful = false;
              result.reason = "Failed to set exposure time";
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

void CameraTis::_Start(
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

void CameraTis::_Stop(
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

}  // namespace camera_tis

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(camera_tis::CameraTis)
