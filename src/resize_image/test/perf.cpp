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

#include <chrono>
#include <iostream>
#include "opencv2/opencv.hpp"

using std::chrono::high_resolution_clock;
using std::chrono::duration_cast;
using std::chrono::milliseconds;

int main()
{
  int rows = 2048, cols = 3072;

  cv::Mat src(rows, cols, CV_8U, cv::Scalar(0));
  cv::Mat dst(rows / 2, cols / 2, CV_8U);

  auto start = high_resolution_clock::now();
  for (auto i = 0; i < 50000; ++i) {
    cv::resize(src, dst, dst.size(), 0, 0);
  }
  auto stop = high_resolution_clock::now();

  auto duration = duration_cast<milliseconds>(stop - start);

  std::cout << "fps: " << 50000 * 1000. / duration.count() << "\n";
  return 0;
}
