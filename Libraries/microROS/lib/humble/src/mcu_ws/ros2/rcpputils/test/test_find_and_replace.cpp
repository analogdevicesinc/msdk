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
#include <string>

#include "rcpputils/find_and_replace.hpp"

TEST(test_find_and_replace, find_and_replace) {
  // Empty input
  {
    auto ret = rcpputils::find_and_replace("", "foo", "bar");
    EXPECT_EQ("", ret);
  }
  // Empty find
  {
    auto ret = rcpputils::find_and_replace("foo", "", "bar");
    EXPECT_EQ("foo", ret);
  }
  // Empty replace
  {
    auto ret = rcpputils::find_and_replace("foo", "foo", "");
    EXPECT_EQ("", ret);
  }
  // Single occurrence
  {
    auto ret = rcpputils::find_and_replace("foo", "foo", "bar");
    EXPECT_EQ("bar", ret);
  }
  // No occurrences
  {
    auto ret = rcpputils::find_and_replace("foo", "bar", "baz");
    EXPECT_EQ("foo", ret);
  }
  // Multiple occurrences
  {
    auto ret = rcpputils::find_and_replace("foobarfoobar", "foo", "baz");
    EXPECT_EQ("bazbarbazbar", ret);
  }
  {
    auto ret = rcpputils::find_and_replace("foofoobar", "foo", "baz");
    EXPECT_EQ("bazbazbar", ret);
  }
  // find == replace
  {
    auto ret = rcpputils::find_and_replace("foobar", "foo", "foo");
    EXPECT_EQ("foobar", ret);
  }
  // find is a substring of replace
  {
    auto ret = rcpputils::find_and_replace("foobar", "foo", "barfoo");
    EXPECT_EQ("barfoobar", ret);
  }
  // replace is a substring of find
  {
    auto ret = rcpputils::find_and_replace("foobar", "foobar", "bar");
    EXPECT_EQ("bar", ret);
  }
}

TEST(test_find_and_replace, find_and_replace_wstring) {
  auto ret = rcpputils::find_and_replace(
    std::wstring(L"foobar"),
    std::wstring(L"foo"),
    std::wstring(L"bar"));
  EXPECT_EQ(std::wstring(L"barbar"), ret);
}

TEST(test_find_and_replace, find_and_replace_various_input_types) {
  rcpputils::find_and_replace(std::string("foo"), "foo", "bar");
  rcpputils::find_and_replace(std::string("foo"), std::string("foo"), "bar");
  rcpputils::find_and_replace(std::string("foo"), "foo", std::string("bar"));
  rcpputils::find_and_replace(std::string("foo"), std::string("foo"), std::string("bar"));

  rcpputils::find_and_replace("foo", std::string("foo"), "bar");
  rcpputils::find_and_replace("foo", std::string("foo"), std::string("bar"));

  rcpputils::find_and_replace("foo", "foo", std::string("bar"));
}
