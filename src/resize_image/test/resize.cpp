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

#undef NDEBUG
#include <cassert>

#include <iostream>
#include "opencv2/opencv.hpp"

int main()
{
  std::vector<uint8_t> buf;
  buf.reserve(1024);
  auto p = buf.data();
  for (auto i = 1; i < 1024; ++i) {
    buf.resize(i);
    assert(p == buf.data());    // No reallocation
  }

  buf.resize(1024);
  assert(p == buf.data());      // No reallocation

  buf.resize(1025);
  assert(p != buf.data());      // Reallocation

  cv::Mat src(100, 100, CV_8UC1, 10);

  {
    cv::Mat dst;
    cv::resize(src, dst, cv::Size(), 0.5, 0.5);
    assert(dst.cols == 50 && dst.rows == 50);
  }

  {
    cv::Mat dst(100, 100, CV_8UC1);
    auto p1 = static_cast<void *>(dst.ptr());
    cv::resize(src, dst, cv::Size(), 0.5, 0.5);   // Reallocation
    auto p2 = static_cast<void *>(dst.ptr());
    assert(dst.cols == 50 && dst.rows == 50);
    assert(p1 != p2);
  }

  {
    cv::Mat dst(50, 50, CV_8UC1);
    auto p1 = static_cast<void *>(dst.ptr());
    cv::resize(src, dst, dst.size(), 0, 0);       // No reallocation
    auto p2 = static_cast<void *>(dst.ptr());
    assert(dst.cols == 50 && dst.rows == 50);
    assert(p1 == p2);
  }

  {
    std::vector<uint8_t> buf(100 * 100, 'a');
    cv::Mat dst(50, 50, CV_8UC1, buf.data());
    auto p1 = static_cast<void *>(dst.ptr());
    cv::resize(src, dst, dst.size(), 0, 0);       // No reallocation
    auto p2 = static_cast<void *>(dst.ptr());
    assert(p1 == p2);
    for (auto i = 0; i < 100 * 100; ++i) {
      if (i < 50 * 50) {
        assert(buf[i] == 10);
      } else {
        assert(buf[i] == 'a');
      }
    }
  }

  {
    std::vector<uint8_t> buf(100 * 100, 'a');
    cv::Mat dst(100, 100, CV_8UC1, buf.data());
    auto p1 = static_cast<void *>(dst.ptr());
    cv::resize(src, dst, cv::Size(), 0.5, 0.5);       // Reallocation
    auto p2 = static_cast<void *>(dst.ptr());
    assert(p1 != p2);
    for (auto i : buf) {
      assert(i == 'a');
    }
  }

  return 0;
}
