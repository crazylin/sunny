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

#include "impl/center.hpp"

using std::chrono::high_resolution_clock;
using std::chrono::duration_cast;
using std::chrono::seconds;

int main()
{
  int rows = 1080, cols = 1920;

  cv::Mat img(rows, cols, CV_8U, cv::Scalar(0)), buf;
  auto col = img.colRange(1000, 1010);
  col = cv::Scalar(100);

  auto start = high_resolution_clock::now();
  for (auto i = 0; i < 10000; ++i) {
    auto p = center(img, buf);
  }
  auto stop = high_resolution_clock::now();

  auto duration = duration_cast<seconds>(stop - start);

  std::cout << "fps: " << 10000 / duration.count() << "\n";
  return 0;
}
