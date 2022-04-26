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
#include <cfloat>

#include "impl/filter.hpp"

int main()
{
  {
    // interpolate
    float p[] = {10, -1, -1, -1, -1, 15};

    filter_length(p, 6, 5, 1., -1);   // gap = 5, step = 1.

    // p[]: 10 11 12 13 14 15
    for (auto i = 0; i < 6; ++i) {
      assert(p[i] == 10 + i);
    }
  }

  {
    // interpolate
    float p[] = {10, -1, -1, -1, -1, 15};

    filter_length(p, 6, 4, 1., -1);   // gap = 4

    // p[]: 10 -1 -1 -1 -1 15
    assert(
      p[0] == 10 &&
      p[1] == -1 &&
      p[2] == -1 &&
      p[3] == -1 &&
      p[4] == -1 &&
      p[5] == 15);
  }

  {
    // interpolate
    float p[] = {10, -1, -1, -1, -1, 15};

    filter_length(p, 6, 5, 1. - DBL_EPSILON, -1);  // step = 1. - DBL_EPSILON

    // p[]: 10 -1 -1 -1 -1 15
    assert(
      p[0] == 10 &&
      p[1] == -1 &&
      p[2] == -1 &&
      p[3] == -1 &&
      p[4] == -1 &&
      p[5] == 15);
  }

  return 0;
}
