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
  cv::Mat img, buf;

  for (size_t i = 1; i < 10; ++i) {
    for (size_t j = 1; j < 10; ++j) {
      img.create(i, j, CV_8U);
      auto pnts = center(img, buf);
      assert(pnts.size() == i);
    }
  }
  return 0;
}
