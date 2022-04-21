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

#ifndef IMPL__PARAMS_HPP_
#define IMPL__PARAMS_HPP_

#include <string>
#include <vector>

/**
 * @brief List of parameter names.
 *
 */
const std::vector<std::string> KEYS = {"ksize", "threshold", "width_min", "width_max"};

/**
 * @brief Group parameters together.
 *
 */
struct Params
{
  int ksize = 5;
  int threshold = 35;
  double width_min = 1.;
  double width_max = 30.;
  double scalar() const
  {
    switch (ksize) {
      case 1:
        return 1.;
      case 3:
        return 1. / 4.;
      case 5:
        return 1. / 48.;
      case 7:
        return 1. / 640.;
      case -1:
        return 1. / 16.;
      default:
        return 0;
    }
  }
};

#endif  // IMPL__PARAMS_HPP_
