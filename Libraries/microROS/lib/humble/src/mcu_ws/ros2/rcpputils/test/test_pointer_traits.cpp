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
#include <memory>

#include "rcpputils/pointer_traits.hpp"

TEST(TestPointerTraits, is_pointer) {
  auto ptr = new int(13);
  auto ptr_ptr = &ptr;
  ASSERT_TRUE(ptr_ptr == &ptr);  // to quiet clang static analysis
  const auto * const cptrc = new int(13);
  auto sptr = std::make_shared<int>(13);
  const auto csptr = std::make_shared<int>(13);
  const volatile auto cvsptr = std::make_shared<int>(13);
  const volatile auto cvsptrc = std::make_shared<int const>(13);
  auto uptr = std::make_unique<int>(13);
  const auto cuptr = std::make_unique<int>(13);
  const volatile auto cvuptr = std::make_unique<int>(13);
  const volatile auto cvuptrc = std::make_unique<const int>(13);

  auto b_ptr = rcpputils::is_pointer<decltype(ptr)>::value;
  auto b_ptr_ptr = rcpputils::is_pointer<decltype(ptr_ptr)>::value;
  auto b_ptr2 = rcpputils::is_pointer<decltype(*ptr_ptr)>::value;
  auto b_cptrc = rcpputils::is_pointer<decltype(cptrc)>::value;
  auto b_sptr = rcpputils::is_pointer<decltype(sptr)>::value;
  auto b_sptr_ref = rcpputils::is_pointer<decltype(sptr) &>::value;
  auto b_csptr = rcpputils::is_pointer<decltype(csptr)>::value;
  auto b_cvsptr = rcpputils::is_pointer<decltype(cvsptr)>::value;
  auto b_cvsptrc = rcpputils::is_pointer<decltype(cvsptrc)>::value;
  auto b_uptr = rcpputils::is_pointer<decltype(uptr)>::value;
  auto b_cuptr = rcpputils::is_pointer<decltype(cuptr)>::value;
  auto b_cvuptr = rcpputils::is_pointer<decltype(cvuptr)>::value;
  auto b_cvuptrc = rcpputils::is_pointer<decltype(cvuptrc)>::value;

  EXPECT_TRUE(b_ptr);
  EXPECT_TRUE(b_ptr_ptr);
  EXPECT_TRUE(b_ptr2);
  EXPECT_TRUE(b_cptrc);
  EXPECT_TRUE(b_sptr);
  EXPECT_TRUE(b_sptr_ref);
  EXPECT_TRUE(b_csptr);
  EXPECT_TRUE(b_cvsptr);
  EXPECT_TRUE(b_cvsptrc);
  EXPECT_TRUE(b_uptr);
  EXPECT_TRUE(b_cuptr);
  EXPECT_TRUE(b_cvuptr);
  EXPECT_TRUE(b_cvuptrc);

  // cleanup raw pointers
  delete ptr;
  delete cptrc;
}

struct POD
{
  int i = 0;
  bool b = false;
};

TEST(TestPointerTraits, is_no_pointer) {
  int i = 13;
  std::string s = "hello world";
  POD pod;

  auto b_i = rcpputils::is_pointer<decltype(i)>::value;
  auto b_s = rcpputils::is_pointer<decltype(s)>::value;
  auto b_pod = rcpputils::is_pointer<decltype(pod)>::value;

  EXPECT_FALSE(b_i);
  EXPECT_FALSE(b_s);
  EXPECT_FALSE(b_pod);
}

TEST(TestPointerTraits, remove_pointer) {
  auto non_ptr = 13;
  auto ptr = new int(13);
  auto ptr_ptr = &ptr;
  ASSERT_TRUE(ptr_ptr == &ptr);  // to quiet clang static analysis
  const auto * const cptrc = new int(13);
  auto sptr = std::make_shared<int>(13);
  const auto csptr = std::make_shared<int>(13);
  const volatile auto cvsptr = std::make_shared<int>(13);
  const volatile auto cvsptrc = std::make_shared<int const>(13);
  auto uptr = std::make_unique<int>(13);
  const auto cuptr = std::make_unique<int>(13);
  const volatile auto cvuptr = std::make_unique<int>(13);
  const volatile auto cvuptrc = std::make_unique<const int>(13);

  auto b_non_ptr = std::is_same<rcpputils::remove_pointer<decltype(non_ptr)>::type, int>::value;
  auto b_ptr = std::is_same<rcpputils::remove_pointer<decltype(ptr)>::type, int>::value;
  auto b_ptr_ptr = std::is_same<rcpputils::remove_pointer<decltype(ptr_ptr)>::type, int>::value;
  auto b_cptrc = std::is_same<rcpputils::remove_pointer<decltype(cptrc)>::type, const int>::value;
  auto b_sptr = std::is_same<rcpputils::remove_pointer<decltype(sptr)>::type, int>::value;
  auto b_csptr = std::is_same<rcpputils::remove_pointer<decltype(csptr)>::type, int>::value;
  auto b_cvsptr = std::is_same<rcpputils::remove_pointer<decltype(cvsptr)>::type, int>::value;
  auto b_cvsptrc =
    std::is_same<rcpputils::remove_pointer<decltype(cvsptrc)>::type, const int>::value;
  auto b_uptr = std::is_same<rcpputils::remove_pointer<decltype(uptr)>::type, int>::value;
  auto b_cuptr = std::is_same<rcpputils::remove_pointer<decltype(cuptr)>::type, int>::value;
  auto b_cvuptr = std::is_same<rcpputils::remove_pointer<decltype(cvuptr)>::type, int>::value;
  auto b_cvuptrc =
    std::is_same<rcpputils::remove_pointer<decltype(cvuptrc)>::type, const int>::value;

  EXPECT_TRUE(b_non_ptr);
  EXPECT_TRUE(b_ptr);
  EXPECT_FALSE(b_ptr_ptr);
  EXPECT_TRUE(b_cptrc);
  EXPECT_TRUE(b_sptr);
  EXPECT_TRUE(b_csptr);
  EXPECT_TRUE(b_cvsptr);
  EXPECT_TRUE(b_cvsptrc);
  EXPECT_TRUE(b_uptr);
  EXPECT_TRUE(b_cuptr);
  EXPECT_TRUE(b_cvuptr);
  EXPECT_TRUE(b_cvuptrc);

  delete ptr;
  delete cptrc;
}
