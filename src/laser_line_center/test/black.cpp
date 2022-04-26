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
  cv::Mat img(100, 100, CV_8U, cv::Scalar(0)), buf;

  auto pnts = center(img, buf);
  assert(pnts.size() == 100);
  for (size_t i = 0; i < pnts.size(); ++i) {
    assert(pnts[i] == -1);
  }

  return 0;
}
