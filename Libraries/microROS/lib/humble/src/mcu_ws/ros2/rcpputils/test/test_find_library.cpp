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

#include <stdlib.h>

#include <string>
#include <utility>

#include "gtest/gtest.h"

#include "rcutils/env.h"
#include "rcpputils/find_library.hpp"

namespace
{

std::pair<const char *, const char *> test_lib_path_and_dir()
{
  const char * test_lib_path{};
  EXPECT_EQ(rcutils_get_env("_TEST_LIBRARY", &test_lib_path), nullptr);
  EXPECT_NE(test_lib_path, nullptr);

  const char * test_lib_dir{};
  EXPECT_EQ(rcutils_get_env("_TEST_LIBRARY_DIR", &test_lib_dir), nullptr);
  EXPECT_NE(test_lib_dir, nullptr);

  return {test_lib_path, test_lib_dir};
}

}  // anonymous namespace

TEST(test_find_library, find_library)
{
  // Get ground-truth values from CTest properties.
  const auto pair = test_lib_path_and_dir();
  const std::string expected_library_path = pair.first;
  const char * test_lib_dir = pair.second;

  // Set our relevant path variable.
  const char * env_var{};
#ifdef _WIN32
  env_var = "PATH";
#elif __APPLE__
  env_var = "DYLD_LIBRARY_PATH";
#else
  env_var = "LD_LIBRARY_PATH";
#endif

#ifdef _WIN32
  EXPECT_EQ(_putenv_s(env_var, test_lib_dir), 0);
#else
  const int override = 1;
  EXPECT_EQ(setenv(env_var, test_lib_dir, override), 0);
#endif

  // Positive test.
  const std::string test_lib_actual = rcpputils::find_library_path("test_library");
  EXPECT_EQ(test_lib_actual, expected_library_path);

  // (Hopefully) Negative test.
  const std::string bad_path = rcpputils::find_library_path(
    "this_is_a_junk_libray_name_please_dont_define_this_if_you_do_then_"
    "you_are_really_naughty");
  EXPECT_EQ(bad_path, "");
}

TEST(test_find_library, library_path)
{
  // Get ground-truth values from CTest properties.
  const auto pair = test_lib_path_and_dir();
  const std::string expected_library_path = pair.first;
  const std::string test_lib_dir = pair.second;

  const std::string test_lib_actual = rcpputils::path_for_library(test_lib_dir, "test_library");
  const std::string bad_path = rcpputils::path_for_library(
    test_lib_dir,
    "highly_unlikely_12_library_34567_name_890.txt.exe");
  EXPECT_EQ(bad_path, "");
}
