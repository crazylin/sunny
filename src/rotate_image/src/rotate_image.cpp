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

#include "rotate_image/rotate_image.hpp"

#include <deque>
#include <memory>

namespace rotate_image
{

using sensor_msgs::msg::Image;

class RotateImage::_Impl
{
public:
  explicit _Impl(RotateImage * ptr)
  : _node(ptr)
  {
  }

  ~_Impl()
  {
  }

  void PushBack(Image::UniquePtr ptr)
  {
    _deq.emplace_back(std::move(ptr));
  }

private:
  RotateImage * _node;
  std::deque<Image::UniquePtr> _deq;
};

RotateImage::RotateImage(const rclcpp::NodeOptions & options)
: Node("rotate_image_node", options)
{
  _pub = this->create_publisher<Image>(_pubName, rclcpp::SensorDataQoS());

  _impl = std::make_unique<_Impl>(this);

  _sub = this->create_subscription<Image>(
    _subName,
    rclcpp::SensorDataQoS(),
    [this](Image::UniquePtr ptr)
    {
      _impl->PushBack(std::move(ptr));
    }
  );
}

RotateImage::~RotateImage()
{
  _sub.reset();
  _impl.reset();
  _pub.reset();

  RCLCPP_INFO(this->get_logger(), "Destroyed successfully");
}

void RotateImage::_InitializeParameters()
{
  // this->declare_parameter("");
}

void RotateImage::_UpdateParameters()
{
  // this->get_parameter("", );
}

}  // namespace rotate_image

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(rotate_image::RotateImage)
