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

#include <exception>
#include <list>
#include <set>
#include <string>
#include <tuple>
#include <vector>

#include "rcpputils/split.hpp"

TEST(test_split, split) {
  {
    auto ret = rcpputils::split("", '/', true);
    EXPECT_EQ(0u, ret.size());
  }
  {
    auto ret = rcpputils::split("", '/', false);
    EXPECT_EQ(0u, ret.size());
  }
  {
    auto ret = rcpputils::split("hello_world", '/', true);
    EXPECT_EQ(1u, ret.size());
    EXPECT_EQ("hello_world", ret[0]);
  }
  {
    auto ret = rcpputils::split("hello_world", '/', false);
    EXPECT_EQ(1u, ret.size());
    EXPECT_EQ("hello_world", ret[0]);
  }
  {
    auto ret = rcpputils::split("hello/world", '/', true);
    EXPECT_EQ(2u, ret.size());
    EXPECT_EQ("hello", ret[0]);
    EXPECT_EQ("world", ret[1]);
  }
  {
    auto ret = rcpputils::split("hello/world", '/', false);
    EXPECT_EQ(2u, ret.size());
    EXPECT_EQ("hello", ret[0]);
    EXPECT_EQ("world", ret[1]);
  }
  {
    auto ret = rcpputils::split("/hello/world", '/', true);
    EXPECT_EQ(2u, ret.size());
    EXPECT_EQ("hello", ret[0]);
    EXPECT_EQ("world", ret[1]);
  }
  {
    auto ret = rcpputils::split("/hello/world", '/', false);
    EXPECT_EQ(3u, ret.size());
    EXPECT_EQ("", ret[0]);
    EXPECT_EQ("hello", ret[1]);
    EXPECT_EQ("world", ret[2]);
  }
  {
    auto ret = rcpputils::split("hello/world/", '/', true);
    EXPECT_EQ(2u, ret.size());
    EXPECT_EQ("hello", ret[0]);
    EXPECT_EQ("world", ret[1]);
  }
  {
    auto ret = rcpputils::split("hello/world/", '/', false);
    EXPECT_EQ(2u, ret.size());
    EXPECT_EQ("hello", ret[0]);
    EXPECT_EQ("world", ret[1]);
  }
  {
    auto ret = rcpputils::split("hello//world", '/', true);
    EXPECT_EQ(2u, ret.size());
    EXPECT_EQ("hello", ret[0]);
    EXPECT_EQ("world", ret[1]);
  }
  {
    auto ret = rcpputils::split("hello//world", '/', false);
    EXPECT_EQ(3u, ret.size());
    EXPECT_EQ("hello", ret[0]);
    EXPECT_EQ("", ret[1]);
    EXPECT_EQ("world", ret[2]);
  }
  {
    auto ret = rcpputils::split("/my/hello/world", '/', true);
    EXPECT_EQ(3u, ret.size());
    EXPECT_EQ("my", ret[0]);
    EXPECT_EQ("hello", ret[1]);
    EXPECT_EQ("world", ret[2]);
  }
  {
    auto ret = rcpputils::split("/my/hello/world", '/', false);
    EXPECT_EQ(4u, ret.size());
    EXPECT_EQ("", ret[0]);
    EXPECT_EQ("my", ret[1]);
    EXPECT_EQ("hello", ret[2]);
    EXPECT_EQ("world", ret[3]);
  }
  {
    auto ret = rcpputils::split("/my//hello//world/", '/', true);
    EXPECT_EQ(3u, ret.size());
    EXPECT_EQ("my", ret[0]);
    EXPECT_EQ("hello", ret[1]);
    EXPECT_EQ("world", ret[2]);
  }
  {
    auto ret = rcpputils::split("/my//hello//world/", '/', false);
    EXPECT_EQ(6u, ret.size());
    EXPECT_EQ("", ret[0]);
    EXPECT_EQ("my", ret[1]);
    EXPECT_EQ("", ret[2]);
    EXPECT_EQ("hello", ret[3]);
    EXPECT_EQ("", ret[4]);
    EXPECT_EQ("world", ret[5]);
  }
}

TEST(test_split, split_backslash) {
  {
    auto ret = rcpputils::split("", '\\', true);
    EXPECT_EQ(0u, ret.size());
  }
  {
    auto ret = rcpputils::split("", '\\', false);
    EXPECT_EQ(0u, ret.size());
  }
  {
    auto ret = rcpputils::split("hello_world", '\\', true);
    EXPECT_EQ(1u, ret.size());
    EXPECT_EQ("hello_world", ret[0]);
  }
  {
    auto ret = rcpputils::split("hello_world", '\\', false);
    EXPECT_EQ(1u, ret.size());
    EXPECT_EQ("hello_world", ret[0]);
  }
  {
    auto ret = rcpputils::split("hello\\world", '\\', true);
    EXPECT_EQ(2u, ret.size());
    EXPECT_EQ("hello", ret[0]);
    EXPECT_EQ("world", ret[1]);
  }
  {
    auto ret = rcpputils::split("hello\\world", '\\', false);
    EXPECT_EQ(2u, ret.size());
    EXPECT_EQ("hello", ret[0]);
    EXPECT_EQ("world", ret[1]);
  }
  {
    auto ret = rcpputils::split("\\hello\\world", '\\', true);
    EXPECT_EQ(2u, ret.size());
    EXPECT_EQ("hello", ret[0]);
    EXPECT_EQ("world", ret[1]);
  }
  {
    auto ret = rcpputils::split("\\hello\\world", '\\', false);
    EXPECT_EQ(3u, ret.size());
    EXPECT_EQ("", ret[0]);
    EXPECT_EQ("hello", ret[1]);
    EXPECT_EQ("world", ret[2]);
  }
  {
    auto ret = rcpputils::split("hello\\world\\", '\\', true);
    EXPECT_EQ(2u, ret.size());
    EXPECT_EQ("hello", ret[0]);
    EXPECT_EQ("world", ret[1]);
  }
  {
    auto ret = rcpputils::split("hello\\world\\", '\\', false);
    EXPECT_EQ(2u, ret.size());
    EXPECT_EQ("hello", ret[0]);
    EXPECT_EQ("world", ret[1]);
  }
  {
    auto ret = rcpputils::split("hello\\\\world", '\\', true);
    EXPECT_EQ(2u, ret.size());
    EXPECT_EQ("hello", ret[0]);
    EXPECT_EQ("world", ret[1]);
  }
  {
    auto ret = rcpputils::split("hello\\\\world", '\\', false);
    EXPECT_EQ(3u, ret.size());
    EXPECT_EQ("hello", ret[0]);
    EXPECT_EQ("", ret[1]);
    EXPECT_EQ("world", ret[2]);
  }
  {
    auto ret = rcpputils::split("\\my\\hello\\world", '\\', true);
    EXPECT_EQ(3u, ret.size());
    EXPECT_EQ("my", ret[0]);
    EXPECT_EQ("hello", ret[1]);
    EXPECT_EQ("world", ret[2]);
  }
  {
    auto ret = rcpputils::split("\\my\\hello\\world", '\\', false);
    EXPECT_EQ(4u, ret.size());
    EXPECT_EQ("", ret[0]);
    EXPECT_EQ("my", ret[1]);
    EXPECT_EQ("hello", ret[2]);
    EXPECT_EQ("world", ret[3]);
  }
  {
    auto ret = rcpputils::split("\\my\\\\hello\\\\world\\", '\\', true);
    EXPECT_EQ(3u, ret.size());
    EXPECT_EQ("my", ret[0]);
    EXPECT_EQ("hello", ret[1]);
    EXPECT_EQ("world", ret[2]);
  }
  {
    auto ret = rcpputils::split("\\my\\\\hello\\\\world\\", '\\', false);
    EXPECT_EQ(6u, ret.size());
    EXPECT_EQ("", ret[0]);
    EXPECT_EQ("my", ret[1]);
    EXPECT_EQ("", ret[2]);
    EXPECT_EQ("hello", ret[3]);
    EXPECT_EQ("", ret[4]);
    EXPECT_EQ("world", ret[5]);
  }
}

TEST(test_split, vector_iterator)
{
  std::string s = "my/hello/world";

  std::vector<std::string> vec = {};
  auto vec_it = std::back_inserter(vec);
  rcpputils::split(s, '/', vec_it);
  EXPECT_EQ("my", vec[0]);
  EXPECT_EQ("hello", vec[1]);
  EXPECT_EQ("world", vec[2]);
}

TEST(test_split, list_iterator)
{
  std::string s = "my/hello/world";

  std::list<std::string> ll = {};
  auto ll_it = std::back_inserter(ll);
  rcpputils::split(s, '/', ll_it);
  EXPECT_EQ("my", ll.front());
  EXPECT_EQ("world", ll.back());
}

TEST(test_split, set_iterator)
{
  std::string s = "wonderful string with wonderful duplicates duplicates";

  std::set<std::string> set = {};
  auto set_it = std::inserter(set, std::begin(set));
  rcpputils::split(s, ' ', set_it);
  EXPECT_EQ(4u, set.size());
}

class TripleExtractor
{
  size_t counter = 0;

public:
  std::string package_name;
  std::string middle_element;
  std::string message_name;

  TripleExtractor & operator=(const std::string & s)
  {
    if (counter >= 3) {
      throw std::out_of_range("triple extractor only can hold three parts");
    }

    if (counter == 0) {
      package_name = s;
    } else if (counter == 1) {
      middle_element = s;
    } else {
      message_name = s;
    }

    counter++;

    return *this;
  }

  std::tuple<std::string, std::string, std::string> get() const
  {
    return std::tie(package_name, middle_element, message_name);
  }
};

TEST(test_split, custom_iterator)
{
  std::string s = "my_pkg/msg/MyMessage";
  TripleExtractor triple_it;
  rcpputils::split(s, '/', triple_it);
  EXPECT_STREQ("my_pkg", triple_it.package_name.c_str());
  EXPECT_STREQ("msg", triple_it.middle_element.c_str());
  EXPECT_STREQ("MyMessage", triple_it.message_name.c_str());

  EXPECT_EQ(std::make_tuple("my_pkg", "msg", "MyMessage"), triple_it.get());
}

TEST(test_split, custom_iterator_exception)
{
  std::string s = "my_pkg/msg/MyMessage/Wrong";
  TripleExtractor triple_it;
  ASSERT_THROW(rcpputils::split(s, '/', triple_it), std::out_of_range);
}
