// Copyright 2020 PAL Robotics S.L.
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
#include <limits>

#include "rcppmath/clamp.hpp"

TEST(test_clamp, test_basic) {
  EXPECT_EQ(rcppmath::clamp(1, 2, 5), 2);
  EXPECT_EQ(rcppmath::clamp(2, 2, 5), 2);
  EXPECT_EQ(rcppmath::clamp(5, 2, 5), 5);
  EXPECT_EQ(rcppmath::clamp(6, 2, 5), 5);
  EXPECT_EQ(rcppmath::clamp(3, 2, 5), 3);
  EXPECT_EQ(rcppmath::clamp(4, 2, 5), 4);
}

TEST(test_clamp, test_cmp) {
  auto cmp = [](const int & a, const int & b)
    {
      return a < b;
    };
  EXPECT_EQ(rcppmath::clamp(1, 2, 5, cmp), 2);
  EXPECT_EQ(rcppmath::clamp(2, 2, 5, cmp), 2);
  EXPECT_EQ(rcppmath::clamp(5, 2, 5, cmp), 5);
  EXPECT_EQ(rcppmath::clamp(6, 2, 5, cmp), 5);
  EXPECT_EQ(rcppmath::clamp(3, 2, 5, cmp), 3);
  EXPECT_EQ(rcppmath::clamp(4, 2, 5, cmp), 4);
}
TEST(test_clamp, test_limits) {
  EXPECT_EQ(rcppmath::clamp(std::numeric_limits<double>::infinity(), 0.0, 1.0), 1.0);
  EXPECT_EQ(rcppmath::clamp(-std::numeric_limits<double>::infinity(), 0.0, 1.0), 0.0);

  // Nan's are not limited by clamp, and return a nan, which is not comparable to itself
  EXPECT_NE(rcppmath::clamp(std::numeric_limits<double>::quiet_NaN(), 0.0, 1.0), 0.0);
  EXPECT_NE(rcppmath::clamp(std::numeric_limits<double>::quiet_NaN(), 0.0, 1.0), 1.0);
  EXPECT_NE(
    rcppmath::clamp(
      std::numeric_limits<double>::quiet_NaN(), 0.0, 1.0),
    std::numeric_limits<double>::quiet_NaN());
}
