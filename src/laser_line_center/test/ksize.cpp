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

#include "impl/center.hpp"

int main()
{
  cv::Mat img(1, 20, CV_8U, cv::Scalar(0)), buf;
  auto col = img.col(10);
  col = cv::Scalar(0xFE);  // Use 0xFE instead of oxff to avoid nasty round.

  int16_t ret;

  cv::Sobel(img, buf, CV_16S, 1, 0, 1, 1);
  ret = 0;
  for (auto i = 0; i < 10; ++i) {
    ret += buf.at<int16_t>(0, i);
  }
  assert(ret == 0xFE);

  cv::Sobel(img, buf, CV_16S, 1, 0, 3, 1 / 4.);
  ret = 0;
  for (auto i = 0; i < 10; ++i) {
    ret += buf.at<int16_t>(0, i);
  }
  assert(ret == 0xFE);

  cv::Sobel(img, buf, CV_16S, 1, 0, 5, 1 / 48.);
  ret = 0;
  for (auto i = 0; i < 10; ++i) {
    ret += buf.at<int16_t>(0, i);
  }
  assert(ret == 0xFE);

  cv::Sobel(img, buf, CV_16S, 1, 0, 7, 1 / 640.);
  ret = 0;
  for (auto i = 0; i < 10; ++i) {
    ret += buf.at<int16_t>(0, i);
  }
  assert(ret == 0xFE);

  cv::Sobel(img, buf, CV_16S, 1, 0, -1, 1 / 16.);
  ret = 0;
  for (auto i = 0; i < 10; ++i) {
    ret += buf.at<int16_t>(0, i);
  }
  assert(ret == 0xFE);

  return 0;
}
