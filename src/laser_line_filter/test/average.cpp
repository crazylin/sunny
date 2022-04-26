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
    float p[] = {10, 10, 10, 10, 10, 10, 10, 10, 10, 10};
    p[4] = 20;
    auto avg = cal_average(p, 10, 2);
    // avg: -1 -1 12 12 12 12 12 10 -1 -1

    filter_average(p, 10, avg, 8);                        // deviate = 8
    assert(p[4] == -1);
  }

  {
    float p[] = {10, 10, 10, 10, 10, 10, 10, 10, 10, 10};
    p[4] = 20;
    auto avg = cal_average(p, 10, 2);
    // avg: -1 -1 12 12 12 12 12 10 -1 -1

    filter_average(p, 10, avg, 8 * (1. + DBL_EPSILON));   // deviate = 8 * (1. + DBL_EPSILON)
    assert(p[4] == 20);
  }

  return 0;
}
