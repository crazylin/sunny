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

#include "impl/center.hpp"

int main()
{
  cv::Mat img(1, 20, CV_8U, cv::Scalar(0)), buf;
  auto col = img.col(10);
  col.setTo(cv::Scalar(0xff));

  auto p = Params();
  int16_t ret;

  p.ksize = 1;
  cv::Sobel(img, buf, CV_16S, 1, 0, p.ksize, p.scalar());
  ret = 0;
  for (auto i = 0; i < 10; ++i) {
    ret += buf.at<int16_t>(0, i);
  }
  assert(ret == 0xff);

  p.ksize = 3;
  cv::Sobel(img, buf, CV_16S, 1, 0, p.ksize, p.scalar());
  ret = 0;
  for (auto i = 0; i < 10; ++i) {
    ret += buf.at<int16_t>(0, i);
  }
  assert(ret == 0xff);

  p.ksize = 5;
  cv::Sobel(img, buf, CV_16S, 1, 0, p.ksize, p.scalar());
  ret = 0;
  for (auto i = 0; i < 10; ++i) {
    ret += buf.at<int16_t>(0, i);
  }
  assert(ret == 0xff);

  p.ksize = 7;
  cv::Sobel(img, buf, CV_16S, 1, 0, p.ksize, p.scalar());
  ret = 0;
  for (auto i = 0; i < 10; ++i) {
    ret += buf.at<int16_t>(0, i);
  }
  assert(ret == 0xff);

  p.ksize = -1;
  cv::Sobel(img, buf, CV_16S, 1, 0, p.ksize, p.scalar());
  ret = 0;
  for (auto i = 0; i < 10; ++i) {
    ret += buf.at<int16_t>(0, i);
  }
  assert(ret == 0xff);

  return 0;
}
