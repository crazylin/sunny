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

#include "params.hpp"

using sensor_msgs::msg::PointCloud2;

/**
 * @brief List of parameter names.
 *
 */
const std::vector<std::string> KEYS = {"enable", "window_size", "gap", "deviate", "step", "length"};

/**
 * @brief Group parameters together.
 *
 */
struct Params
{
  bool enable = false;
  int ws = 10;
  int gap = 5;
  double dev = 5.;
  double step = 2.;
  int length = 30;
};

/**
 * @brief The algorithm to filter out noise points.
 *
 * For more details of the algorithm, refer to the README.md.
 * @param ptr The input point cloud data.
 * @param pms Parameters group together.
 * @return PointCloud2::UniquePtr Point cloud message to publish.
 */
PointCloud2::UniquePtr filter(PointCloud2::UniquePtr ptr, const Params & pms = Params())
{
  if (pms.enable == false) {
    return ptr;
  }

  auto num = static_cast<int>(ptr->width);
  if (ptr->header.frame_id == "-1" || num == 0) {
    return ptr;
  } else {
    std::vector<float> buf;
    buf.resize(num, -1);
    auto p = reinterpret_cast<float *>(ptr->data.data());

    for (int i = pms.ws; i < num - pms.ws; ++i) {
      if (p[i] < 0) {
        continue;
      }

      float sum = 0;
      int hit = 0;
      for (auto j = -pms.ws; j <= pms.ws; ++j) {
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
      if (abs(p[i] - buf[i]) > pms.dev) {
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
        if (j - f <= pms.gap && abs(p[j] - p[f]) / (j - f) < pms.step) {
          f = j;
          ++j;
        } else {
          break;
        }
      }
      if (f - i < pms.length) {
        for (auto k = i; k <= f; ++k) {
          p[k] = -1;
        }
      } else {
        i = j;
      }
    }

    return ptr;
  }
}

#endif  // IMPL__FILTER_HPP_
