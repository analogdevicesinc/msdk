// Copyright 2020 Open Source Robotics Foundation, Inc.
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

#include "rcpputils/scope_exit.hpp"

TEST(test_scope_exit, should_run) {
  bool called = false;

  {
    auto on_exit =
      rcpputils::make_scope_exit(
      [&called]() {
        called = true;
      });
    (void)on_exit;
  }

  EXPECT_TRUE(called);
}

TEST(test_scope_exit, should_not_run) {
  bool called = false;

  {
    auto on_exit =
      rcpputils::make_scope_exit(
      [&called]() {
        called = true;
      });
    on_exit.cancel();
  }

  EXPECT_FALSE(called);
}

TEST(test_scope_exit, code_types) {
  auto code = []() {};
  rcpputils::make_scope_exit(code);

  const auto const_code = []() {};
  rcpputils::make_scope_exit(const_code);

  struct NonCopyableCode
  {
    NonCopyableCode() = default;
    NonCopyableCode(const NonCopyableCode &) = delete;
    NonCopyableCode(NonCopyableCode &&) = default;
    void operator()() {}
  };
  rcpputils::make_scope_exit(NonCopyableCode());
}
