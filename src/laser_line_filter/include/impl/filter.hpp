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

#ifndef IMPL__FILTER_HPP_
#define IMPL__FILTER_HPP_

#include <vector>

using sensor_msgs::msg::PointCloud2;


/**
 * @brief The algorithm to filter out noise points.
 *
 * For more details of the algorithm, refer to the README.md.
 * @param ptr The input point cloud data.
 * @param pms Parameters group together.
 * @return PointCloud2::UniquePtr Point cloud message to publish.
 */
PointCloud2::UniquePtr filter(
  PointCloud2::UniquePtr ptr,
  bool enable = false,
  int window_size = 10,
  int gap = 5,
  double deviate = 5.,
  double step = 2.,
  int length = 30)
{
  if (ptr->header.frame_id == "-1" || ptr->data.empty()) {return ptr;}
  if (enable == false) {return ptr;}

  auto num = static_cast<int>(ptr->width);
  std::vector<float> buf;
  buf.resize(num, -1);
  auto p = reinterpret_cast<float *>(ptr->data.data());
  for (int i = window_size; i < num - window_size; ++i) {
    if (p[i] < 0) {
      continue;
    }

    float sum = 0;
    int hit = 0;
    for (auto j = -window_size; j <= window_size; ++j) {
      if (p[i + j] < 0) {
        continue;
      }
      sum += p[i + j];
      ++hit;
    }
    buf[i] = sum / hit;
  }

  // filter by diff with average
  for (int i = 0; i < num; ++i) {
    if (p[i] < 0 || buf[i] < 0) {
      continue;
    }
    if (abs(p[i] - buf[i]) > deviate) {
      p[i] = -1;
    }
  }

  // filter by length
  auto i = 0;
  while (i < num) {
    if (p[i] < 0) {
      ++i;
      continue;
    }
    auto f = i;
    auto j = f + 1;
    while (j < num) {
      if (p[j] < 0) {
        ++j;
        continue;
      }
      if (j - f <= gap && abs(p[j] - p[f]) / (j - f) < step) {
        f = j;
        ++j;
      } else {
        break;
      }
    }
    if (f - i < length) {
      for (auto k = i; k <= f; ++k) {
        p[k] = -1;
      }
    } else {
      i = j;
    }
  }

  return ptr;
}

#endif  // IMPL__FILTER_HPP_
