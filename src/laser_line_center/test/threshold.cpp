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
  col = cv::Scalar(50);

  auto p = Params();
  p.ksize = 1;

  p.threshold = 50;   // Threshold set to 50, shall pass.
  auto pnts = center(img, buf, p);
  assert(pnts[0] == 10);

  p.threshold = 51;   // Threshold set to 51, shall be filtered.
  pnts = center(img, buf, p);
  assert(pnts[0] == -1);

  return 0;
}
