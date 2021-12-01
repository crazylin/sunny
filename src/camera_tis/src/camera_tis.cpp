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
      ptr->height = 480;
      ptr->width = 640;
      ptr->encoding = "mono8";
      ptr->is_bigendian = false;
      ptr->step = 640;
      ptr->data.resize(640 * 480);
      memcpy(ptr->data.data(), data, 640 * 480);
      node->Publish(ptr);

      gst_buffer_unmap(buffer, &info);
      gst_video_info_free(video_info);
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
    gst_debug_set_default_threshold(GST_LEVEL_WARNING);
    gst_init(NULL, NULL);
    const char * pipeline_str =
      "tcambin name=source ! video/x-raw,format=GRAY8,width=640,height=480,framerate=30/1 ! videoconvert ! appsink name=sink";  // NOLINT
    GError * err = NULL;
    _pipeline = gst_parse_launch(pipeline_str, &err);
    if (_pipeline == NULL) {
      throw std::runtime_error("TIS pipeline fail");
    }
    GstElement * source = gst_bin_get_by_name(GST_BIN(_pipeline), "source");

    GValue val = {};
    const char * serial = NULL;
    g_value_init(&val, G_TYPE_STRING);
    g_value_set_static_string(&val, serial);

    g_object_set_property(G_OBJECT(source), "serial", &val);
    gst_object_unref(source);

    /* retrieve the appsink from the pipeline */
    GstElement * sink = gst_bin_get_by_name(GST_BIN(_pipeline), "sink");

    // tell appsink to notify us when it receives an image
    g_object_set(G_OBJECT(sink), "emit-signals", TRUE, NULL);

    // tell appsink what function to call when it notifies us
    g_signal_connect(sink, "new-sample", G_CALLBACK(callback), _node);

    gst_object_unref(sink);

    block_until_playing(_pipeline);

    GValue expo_auto = G_VALUE_INIT, gain_auto = G_VALUE_INIT;
    g_value_init(&expo_auto, G_TYPE_BOOLEAN);
    g_value_init(&gain_auto, G_TYPE_BOOLEAN);

    g_value_set_boolean(&expo_auto, FALSE);
    g_value_set_boolean(&gain_auto, FALSE);

    tcam_prop_set_tcam_property(TCAM_PROP(source), "Exposure Auto", &expo_auto);
    tcam_prop_set_tcam_property(TCAM_PROP(source), "Gain Auto", &gain_auto);

    g_value_unset(&expo_auto);
    g_value_unset(&gain_auto);

    GValue set_expo = G_VALUE_INIT;
    g_value_init(&set_expo, G_TYPE_INT);
    g_value_set_int(&set_expo, 6000);
    tcam_prop_set_tcam_property(TCAM_PROP(source), "Exposure Time (us)", &set_expo);
    g_value_unset(&set_expo);
  }

  ~_Impl()
  {
    // gst_object_unref(_pipeline);
  }

  void Start()
  {
    gst_element_set_state(_pipeline, GST_STATE_PLAYING);
  }

  void Stop()
  {
    gst_element_set_state(_pipeline, GST_STATE_NULL);
  }

private:
  CameraTis * _node;
  GstElement * _pipeline;
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
