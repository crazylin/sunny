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

#include <cmath>
#include <vector>

std::vector<float> cal_average(const float * ptr, int width, int window_size)
{
  std::vector<float> avg;
  avg.resize(width, -1);
  for (auto i = window_size; i < width - window_size; ++i) {
    if (ptr[i] < 0) {continue;}

    float sum = 0;
    int hit = 0;
    for (auto j = -window_size; j <= window_size; ++j) {
      if (ptr[i + j] < 0) {continue;}

      sum += ptr[i + j];
      ++hit;
    }
    avg[i] = sum / hit;
  }
  return avg;
}

void filter_average(float * ptr, int width, const std::vector<float> & avg, double deviate)
{
  for (auto i = 0; i < width; ++i) {
    if (ptr[i] < 0 || avg[i] < 0) {continue;}

    if (abs(ptr[i] - avg[i]) > deviate) {
      ptr[i] = -1;
    }
  }
}

void filter_length(float * ptr, int width, int gap, double step, int length)
{
  // start point i
  auto i = 0;
  while (i < width) {
    if (ptr[i] < 0) {
      ++i;
      continue;
    }
    // front point f as first non -1 start point
    auto f = i;
    // move point j start from f + 1
    auto j = f + 1;
    while (j < width) {
      if (ptr[j] < 0) {
        ++j;
        continue;
      }
      // move to first non -1
      if (j - f <= gap && abs(ptr[j] - ptr[f]) / (j - f) < step) {
        // interpolate in between
        if (j - f > 1) {
          auto s = (ptr[j] - ptr[f]) / (j - f);
          for (auto k = 1; k < j - f; ++k) {
            ptr[f + k] = ptr[f] + s * k;
          }
        }
        // move forward front point and continue move j forward
        f = j;
        ++j;
      } else {
        break;
      }
    }

    // calculate the distance between front and start points
    if (f - i < length) {
      // disable all in between if length criteria fails.
      for (auto k = i; k <= f; ++k) {
        ptr[k] = -1;
      }
    }

    // move start point to j
    i = j;
  }
}

/**
 * @brief The algorithm to filter out noise points.
 *
 * For more details of the algorithm, refer to the README.md.
 * @param ptr The input point cloud data.
 * @param pms Parameters group together.
 * @return PointCloud2::UniquePtr Point cloud message to publish.
 */
void filter(
  float * ptr,
  int width,
  int window_size = 10,
  int gap = 5,
  double deviate = 5.,
  double step = 2.,
  int length = 30)
{
  auto buf = cal_average(ptr, width, window_size);

  filter_average(ptr, width, buf, deviate);

  filter_length(ptr, width, gap, step, length);
}

#endif  // IMPL__FILTER_HPP_
