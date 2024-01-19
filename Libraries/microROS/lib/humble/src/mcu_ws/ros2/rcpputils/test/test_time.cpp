// Copyright 2021 Open Source Robotics Foundation, Inc.
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

#include <gtest/gtest.h>

#include <rcpputils/time.hpp>

TEST(test_time, test_convert_to_nanoseconds) {
  rcutils_duration_value_t expect_value = RCUTILS_S_TO_NS(5 * 60);  // 5 minutes
  rcutils_duration_value_t cast_val = 0;
  EXPECT_NO_THROW(
    cast_val = rcpputils::convert_to_nanoseconds(std::chrono::duration<double>(5 * 60)).count());
  EXPECT_EQ(cast_val, expect_value);

  EXPECT_THROW(
    rcpputils::convert_to_nanoseconds(std::chrono::hours(10000000)),
    std::invalid_argument);
}
