// Copyright 2019 Open Source Robotics Foundation, Inc.
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

#include <list>
#include <string>
#include <vector>

#include "rcpputils/join.hpp"


TEST(test_join, join_strings_vector) {
  {
    const std::vector<std::string> zero_elements;
    EXPECT_EQ("", rcpputils::join(zero_elements, ""));
    EXPECT_EQ("", rcpputils::join(zero_elements, ", "));
  }
  {
    const std::vector<std::string> one_element{"foo"};
    EXPECT_EQ("foo", rcpputils::join(one_element, ""));
    EXPECT_EQ("foo", rcpputils::join(one_element, ", "));
  }
  {
    const std::vector<std::string> many_elements{"foo", "bar", "baz"};
    EXPECT_EQ("foobarbaz", rcpputils::join(many_elements, ""));
    EXPECT_EQ("foo, bar, baz", rcpputils::join(many_elements, ", "));
  }
}

TEST(test_join, join_strings_list) {
  {
    const std::list<std::string> zero_elements;
    EXPECT_EQ("", rcpputils::join(zero_elements, ""));
    EXPECT_EQ("", rcpputils::join(zero_elements, ", "));
  }
  {
    const std::list<std::string> one_element{"foo"};
    EXPECT_EQ("foo", rcpputils::join(one_element, ""));
    EXPECT_EQ("foo", rcpputils::join(one_element, ", "));
  }
  {
    const std::list<std::string> many_elements{"foo", "bar", "baz"};
    EXPECT_EQ("foobarbaz", rcpputils::join(many_elements, ""));
    EXPECT_EQ("foo, bar, baz", rcpputils::join(many_elements, ", "));
  }
}

TEST(test_join, join_ints_vector) {
  {
    const std::vector<int> zero_elements;
    EXPECT_EQ("", rcpputils::join(zero_elements, ""));
    EXPECT_EQ("", rcpputils::join(zero_elements, ", "));
  }
  {
    const std::vector<int> one_element{1};
    EXPECT_EQ("1", rcpputils::join(one_element, ""));
    EXPECT_EQ("1", rcpputils::join(one_element, ", "));
  }
  {
    const std::vector<int> many_elements{1, 2, 3};
    EXPECT_EQ("123", rcpputils::join(many_elements, ""));
    EXPECT_EQ("1, 2, 3", rcpputils::join(many_elements, ", "));
  }
}

TEST(test_join, join_ints_list) {
  {
    const std::list<int> zero_elements;
    EXPECT_EQ("", rcpputils::join(zero_elements, ""));
    EXPECT_EQ("", rcpputils::join(zero_elements, ", "));
  }
  {
    const std::list<int> one_element{1};
    EXPECT_EQ("1", rcpputils::join(one_element, ""));
    EXPECT_EQ("1", rcpputils::join(one_element, ", "));
  }
  {
    const std::list<int> many_elements{1, 2, 3};
    EXPECT_EQ("123", rcpputils::join(many_elements, ""));
    EXPECT_EQ("1, 2, 3", rcpputils::join(many_elements, ", "));
  }
}
