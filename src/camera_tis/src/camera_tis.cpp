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

#include <deque>
#include <exception>
#include <future>
#include <map>
#include <memory>
#include <utility>
#include <vector>

#include "opencv2/opencv.hpp"

namespace camera_tis
{

using sensor_msgs::msg::Image;

const auto WIDTH = 3072, HEIGHT = 2048, FPS = 60;

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
    node->PushBackImage(sample);
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
      "! queue max-size-buffers=2 leaky=downstream "
      "! videoconvert "
      "! appsink name=sink emit-signals=true sync=false drop=true max-buffers=5";
    GError * err = NULL;
    _pipeline = gst_parse_launch(pipeline_str, &err);
    if (_pipeline == NULL) {
      throw std::runtime_error("TIS pipeline fail");
    }

    _SetProperty("Exposure Auto", false);
    _SetProperty("Gain Auto", false);
    _SetProperty("Exposure Time (us)", _expo);
    _SetProperty("Brightness", 0);

    _SetCaps("GRAY8", WIDTH, HEIGHT, FPS);

    /* retrieve the appsink from the pipeline */
    GstElement * sink = gst_bin_get_by_name(GST_BIN(_pipeline), "sink");

    // tell appsink what function to call when it notifies us
    g_signal_connect(sink, "new-sample", G_CALLBACK(callback), _node);

    gst_object_unref(sink);

    block_until_playing(_pipeline);

    for (int i = 0; i < _workers; ++i) {
      _threads.push_back(std::thread(&_Impl::_Worker, this));
    }
    _threads.push_back(std::thread(&_Impl::_Manager, this));
    RCLCPP_INFO(_node->get_logger(), "Employ %d workers successfully", _workers);
  }

  ~_Impl()
  {
    // gst_element_set_state(_pipeline, GST_STATE_NULL);
    _images_con.notify_all();
    _futures_con.notify_one();
    for (auto & t : _threads) {
      t.join();
    }
    gst_object_unref(_pipeline);
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
    _node->declare_parameter("workers", _workers);
  }

  void _UpdateParameters()
  {
    _node->get_parameter("exposure_time", _expo);
    _node->get_parameter("power", _power);
    _node->get_parameter("workers", _workers);
    _deque_size = _workers + 1;
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

  void PushBackImage(GstSample * sample)
  {
    std::unique_lock<std::mutex> lk(_images_mut);
    _images.emplace_back(sample);
    auto s = static_cast<int>(_images.size());
    if (s > _deque_size) {
      sample = _images.front();
      _images.pop_front();
      gst_sample_unref(sample);
    }
    lk.unlock();
    _images_con.notify_all();
  }

  void PushBackFuture(std::future<Image::UniquePtr> f)
  {
    std::unique_lock<std::mutex> lk(_futures_mut);
    _futures.emplace_back(std::move(f));
    lk.unlock();
    _futures_con.notify_one();
  }

  void _Manager()
  {
    while (rclcpp::ok()) {
      std::unique_lock<std::mutex> lk(_futures_mut);
      if (_futures.empty() == false) {
        auto f = std::move(_futures.front());
        _futures.pop_front();
        lk.unlock();
        auto ptr = f.get();
        static int frame_id = 0;
        ptr->header.frame_id = std::to_string(frame_id++);
        _node->Publish(ptr);
      } else {
        _futures_con.wait(lk);
      }
    }
  }

  void _Worker()
  {
    while (rclcpp::ok()) {
      std::unique_lock<std::mutex> lk(_images_mut);
      if (_images.empty() == false) {
        auto sample = _images.front();
        _images.pop_front();
        std::promise<Image::UniquePtr> prom;
        PushBackFuture(prom.get_future());
        lk.unlock();
        auto ptr = std::make_unique<Image>();
        GstBuffer * buffer = gst_sample_get_buffer(sample);
        GstMapInfo info;  // contains the actual image
        if (gst_buffer_map(buffer, &info, GST_MAP_READ)) {
          GstVideoInfo * video_info = gst_video_info_new();
            if (!gst_video_info_from_caps(video_info, gst_sample_get_caps(sample))) {
              gst_sample_unref(sample);
              RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to parse video info");
              rclcpp::shutdown();
              break;
            }
            /* Get a pointer to the image data */
          cv::Mat src(WIDTH, HEIGHT, CV_8UC1, CV_8UC1, info.data);
          ptr->header.stamp = _node->now();
          ptr->height = HEIGHT / 2;
          ptr->width = WIDTH / 2;
          ptr->encoding = "mono8";
          ptr->is_bigendian = false;
          ptr->step = WIDTH / 2;
          ptr->data.resize(HEIGHT * WIDTH / 4);
          cv::Mat dst(ptr->width, ptr->height, CV_8UC1, ptr->data.data());
          cv::resize(img, dst, cv::Size());

          gst_buffer_unmap(buffer, &info);
          gst_video_info_free(video_info);
        }
        prom.set_value(std::move(ptr));
        gst_sample_unref(sample);
      } else {
        _images_con.wait(lk);
      }
    }
  }

private:
  CameraTis * _node;
  int _workers = 1;
  int _deque_size;

  GstElement * _pipeline;
  int _expo = 1000;
  bool _power = false;

  std::mutex _images_mut;
  std::condition_variable _images_con;
  std::deque<GstSample *> _images;

  std::mutex _futures_mut;
  std::condition_variable _futures_con;
  std::deque<std::future<Image::UniquePtr>> _futures;

  std::vector<std::thread> _threads;
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

void CameraTis::PushBackImage(void * sample)
{
  _impl->PushBackImage(static_cast<GstSample *>(sample));
}

}  // namespace camera_tis

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(camera_tis::CameraTis)
