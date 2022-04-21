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

#ifndef IMPL__TO_PC2_HPP_
#define IMPL__TO_PC2_HPP_

#include <memory>
#include <vector>

#include "sensor_msgs/msg/point_cloud2.hpp"

using sensor_msgs::msg::PointCloud2;

/**
   * @brief Construct ROS point cloud message from vector of floats.
   *
   * @param pnts A sequence of floats as points' row coordinate.
   * @return PointCloud2::UniquePtr Point cloud message to publish.
   */
PointCloud2::UniquePtr to_pc2(const std::vector<float> & pnts)
{
  auto ptr = std::make_unique<PointCloud2>();
  if (pnts.empty()) {return ptr;}

  auto num = pnts.size();

  ptr->height = 1;
  ptr->width = num;

  ptr->fields.resize(1);

  ptr->fields[0].name = "u";
  ptr->fields[0].offset = 0;
  ptr->fields[0].datatype = 7;
  ptr->fields[0].count = 1;

  ptr->is_bigendian = false;
  ptr->point_step = 4;
  ptr->row_step = num * 4;

  ptr->data.resize(num * 4);

  ptr->is_dense = true;

  memcpy(ptr->data.data(), pnts.data(), num * 4);

  return ptr;
}

#endif  // IMPL__TO_PC2_HPP_
