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

#include <fstream>
#include <string>

#include "rcpputils/filesystem_helper.hpp"
#include "rcpputils/env.hpp"

#ifdef _WIN32
static constexpr const bool is_win32 = true;
#else
static constexpr const bool is_win32 = false;
#endif

using path = rcpputils::fs::path;

std::string build_extension_path()
{
  return is_win32 ? R"(C:\foo\bar\baz.yml)" : "/bar/foo/baz.yml";
}

std::string build_double_extension_path()
{
  return is_win32 ? R"(C:\bar\baz.bar.yml)" : "/foo/baz.bar.yml";
}

std::string build_no_extension_path()
{
  return is_win32 ? R"(C:\bar\baz)" : "/bar/baz";
}

std::string build_directory_path()
{
  return is_win32 ? R"(.\test_folder)" : R"(./test_folder)";
}

TEST(TestFilesystemHelper, join_path)
{
  {
    auto p = path("foo") / path("bar");
    if (is_win32) {
      EXPECT_EQ("foo\\bar", p.string());
    } else {
      EXPECT_EQ("foo/bar", p.string());
    }
  }

  if (is_win32) {
    auto p = path("foo") / path("C:\\bar");
    EXPECT_EQ("C:\\bar", p.string());
  } else {
    auto p = path("foo") / path("/bar");
    EXPECT_EQ("/bar", p.string());
  }

  {
    // Just expect these join operations to be allowed with const paths
    const auto p1 = path("foo");
    auto p1_baz = p1 / "baz";

    const auto p2 = path("bar");
    auto p1_2 = p1 / p2;
  }
}

TEST(TestFilesystemHelper, parent_path)
{
  {
    auto p = path("my") / path("path");
    EXPECT_EQ(p.parent_path().string(), path("my").string());
  }
  {
    auto p = path("foo");
    EXPECT_EQ(p.parent_path().string(), ".");
  }
  {
    if (is_win32) {
      {
        auto p = path("C:\\foo");
        EXPECT_EQ(p.parent_path().string(), "C:\\");
      }
      {
        auto p = path("\\foo");
        EXPECT_EQ(p.parent_path().string(), "\\");
      }
    } else {
      auto p = path("/foo");
      EXPECT_EQ(p.parent_path().string(), "/");
    }
  }
  {
    if (is_win32) {
      {
        auto p = path("C:\\");
        EXPECT_EQ(p.parent_path().string(), "C:\\");
      }
      {
        auto p = path("\\");
        EXPECT_EQ(p.parent_path().string(), "\\");
      }
    } else {
      auto p = path("/");
      EXPECT_EQ(p.parent_path().string(), "/");
    }
  }
  {
    auto p = path("");
    EXPECT_EQ(p.parent_path().string(), "");
  }
}

TEST(TestFilesystemHelper, to_native_path)
{
  {
    auto p = path("/foo/bar/baz");
    if (is_win32) {
      EXPECT_EQ("\\foo\\bar\\baz", p.string());
    } else {
      EXPECT_EQ("/foo/bar/baz", p.string());
    }
  }
  {
    auto p = path("\\foo\\bar\\baz");
    if (is_win32) {
      EXPECT_EQ("\\foo\\bar\\baz", p.string());
    } else {
      EXPECT_EQ("/foo/bar/baz", p.string());
    }
  }
  {
    auto p = path("/foo//bar/baz");
    if (is_win32) {
      EXPECT_EQ("\\foo\\\\bar\\baz", p.string());
    } else {
      EXPECT_EQ("/foo//bar/baz", p.string());
    }
  }
  {
    auto p = path("\\foo\\\\bar\\baz");
    if (is_win32) {
      EXPECT_EQ("\\foo\\\\bar\\baz", p.string());
    } else {
      EXPECT_EQ("/foo//bar/baz", p.string());
    }
  }
}

TEST(TestFilesystemHelper, is_absolute)
{
  if (is_win32) {
    {
      auto p = path("C:\\foo\\bar\\baz");
      EXPECT_TRUE(p.is_absolute());
    }
    {
      auto p = path("D:\\foo\\bar\\baz");
      EXPECT_TRUE(p.is_absolute());
    }
    {
      auto p = path("C:/foo/bar/baz");
      EXPECT_TRUE(p.is_absolute());
    }
    {
      auto p = path("\\foo\\bar\\baz");
      EXPECT_TRUE(p.is_absolute());
    }
    {
      auto p = path("/foo/bar/baz");
      EXPECT_TRUE(p.is_absolute());
    }
    {
      auto p = path("foo/bar/baz");
      EXPECT_FALSE(p.is_absolute());
    }
    {
      auto p = path("C:");
      EXPECT_FALSE(p.is_absolute());
    }
    {
      auto p = path("C");
      EXPECT_FALSE(p.is_absolute());
    }
    {
      auto p = path("");
      EXPECT_FALSE(p.is_absolute());
    }
  } else {
    {
      auto p = path("/foo/bar/baz");
      EXPECT_TRUE(p.is_absolute());
    }
    {
      auto p = path("foo/bar/baz");
      EXPECT_FALSE(p.is_absolute());
    }
    {
      auto p = path("f");
      EXPECT_FALSE(p.is_absolute());
    }
    {
      auto p = path("");
      EXPECT_FALSE(p.is_absolute());
    }
  }
}

TEST(TestFilesystemHelper, correct_extension)
{
  {
    auto p = path(build_extension_path());
    auto ext = p.extension();
    EXPECT_EQ(".yml", ext.string());
  }
  {
    auto p = path(build_double_extension_path());
    auto ext = p.extension();
    EXPECT_EQ(".yml", ext.string());
  }
  {
    auto p = path(build_no_extension_path());
    auto ext = p.extension();
    EXPECT_EQ("", ext.string());
  }
}

TEST(TestFilesystemHelper, is_empty)
{
  auto p_no_arg = path();
  EXPECT_TRUE(p_no_arg.empty());

  auto p_empty_string = path("");
  EXPECT_TRUE(p_empty_string.empty());
}

TEST(TestFilesystemHelper, exists)
{
  {
    auto p = path("");
    EXPECT_FALSE(p.exists());
  }
  {
    auto p = path(".");
    EXPECT_TRUE(p.exists());
  }
  {
    auto p = path("..");
    EXPECT_TRUE(p.exists());
  }
  {
    if (is_win32) {
      auto p = path("\\");
      EXPECT_TRUE(p.exists());
    } else {
      auto p = path("/");
      EXPECT_TRUE(p.exists());
    }
  }
}

/**
 * Test filesystem manipulation API.
 *
 * NOTE: expects the current directory to be write-only, else test will fail.
 *
 */
TEST(TestFilesystemHelper, filesystem_manipulation)
{
  auto dir = path(build_directory_path());
  (void)rcpputils::fs::remove(dir);
  EXPECT_FALSE(rcpputils::fs::exists(dir));
  EXPECT_TRUE(rcpputils::fs::create_directories(dir));
  EXPECT_TRUE(rcpputils::fs::exists(dir));
  EXPECT_TRUE(rcpputils::fs::is_directory(dir));
  EXPECT_FALSE(rcpputils::fs::is_regular_file(dir));

  auto file = dir / "test_file.txt";
  uint64_t expected_file_size = 0;
  {
    std::ofstream output_buffer{file.string()};
    output_buffer << "test";
    expected_file_size = static_cast<uint64_t>(output_buffer.tellp());
  }

  EXPECT_TRUE(rcpputils::fs::exists(file));
  EXPECT_TRUE(rcpputils::fs::is_regular_file(file));
  EXPECT_FALSE(rcpputils::fs::is_directory(file));
  EXPECT_FALSE(rcpputils::fs::create_directories(file));
  EXPECT_GE(rcpputils::fs::file_size(file), expected_file_size);
  EXPECT_THROW(rcpputils::fs::file_size(dir), std::system_error) <<
    "file_size is only applicable for files!";
  EXPECT_FALSE(rcpputils::fs::remove(dir));
  EXPECT_TRUE(rcpputils::fs::remove(file));
  EXPECT_THROW(rcpputils::fs::file_size(file), std::system_error);
  EXPECT_TRUE(rcpputils::fs::remove(dir));
  EXPECT_FALSE(rcpputils::fs::exists(file));
  EXPECT_FALSE(rcpputils::fs::exists(dir));
  auto temp_dir = rcpputils::fs::temp_directory_path();
  temp_dir = temp_dir / "rcpputils" / "test_folder";
  EXPECT_FALSE(rcpputils::fs::exists(temp_dir));
  EXPECT_TRUE(rcpputils::fs::create_directories(temp_dir));
  EXPECT_TRUE(rcpputils::fs::exists(temp_dir));
  EXPECT_TRUE(rcpputils::fs::remove(temp_dir));
  EXPECT_TRUE(rcpputils::fs::remove(temp_dir.parent_path()));

  // Remove empty directory
  EXPECT_FALSE(rcpputils::fs::exists(dir));
  EXPECT_TRUE(rcpputils::fs::create_directories(dir));
  EXPECT_TRUE(rcpputils::fs::exists(dir));
  EXPECT_TRUE(rcpputils::fs::is_directory(dir));
  EXPECT_TRUE(rcpputils::fs::remove_all(dir));
  EXPECT_FALSE(rcpputils::fs::is_directory(dir));

  // Remove non-existing directory
  EXPECT_FALSE(rcpputils::fs::remove_all(rcpputils::fs::path("some") / "nonsense" / "dir"));

  EXPECT_FALSE(rcpputils::fs::exists(dir));
  EXPECT_TRUE(rcpputils::fs::create_directories(dir));

  // Remove single file with remove_all
  file = dir / "remove_all_single_file.txt";
  {
    std::ofstream output_buffer{file.string()};
    output_buffer << "some content";
  }
  ASSERT_TRUE(rcpputils::fs::exists(file));
  ASSERT_TRUE(rcpputils::fs::is_regular_file(file));
  EXPECT_TRUE(rcpputils::fs::remove_all(file));

  // Remove directory and its content
  EXPECT_TRUE(rcpputils::fs::exists(dir));
  EXPECT_TRUE(rcpputils::fs::is_directory(dir));

  auto num_files = 10u;
  for (auto i = 0u; i < num_files; ++i) {
    std::string file_name = std::string("test_file") + std::to_string(i) + ".txt";
    auto file = dir / file_name;
    {
      std::ofstream output_buffer{file.string()};
      output_buffer << "test" << i;
    }
  }

  // remove shall fail given that directory is not empty
  ASSERT_FALSE(rcpputils::fs::remove(dir));

  EXPECT_TRUE(rcpputils::fs::remove_all(dir));

  for (auto i = 0u; i < num_files; ++i) {
    std::string file_name = std::string("test_file") + std::to_string(i) + ".txt";
    auto file = dir / file_name;
    ASSERT_FALSE(rcpputils::fs::exists(file));
  }
  ASSERT_FALSE(rcpputils::fs::exists(dir));

  // Empty path/directory cannot be created
  EXPECT_FALSE(rcpputils::fs::create_directories(rcpputils::fs::path("")));
}

TEST(TestFilesystemHelper, remove_extension)
{
  auto p = path("foo.txt");
  p = rcpputils::fs::remove_extension(p.string());
  EXPECT_EQ("foo", p.string());
}

TEST(TestFilesystemHelper, remove_extensions)
{
  auto p = path("foo.txt.compress");
  p = rcpputils::fs::remove_extension(p, 2);
  EXPECT_EQ("foo", p.string());
}

TEST(TestFilesystemHelper, remove_extensions_overcount)
{
  auto p = path("foo.txt.compress");
  p = rcpputils::fs::remove_extension(p, 4);
  EXPECT_EQ("foo", p.string());
}

TEST(TestFilesystemHelper, remove_extension_no_extension)
{
  auto p = path("foo");
  p = rcpputils::fs::remove_extension(p);
  EXPECT_EQ("foo", p.string());
}

TEST(TestFilesystemHelper, get_cwd)
{
  std::string expected_dir = rcpputils::get_env_var("EXPECTED_WORKING_DIRECTORY");
  auto p = rcpputils::fs::current_path();
  EXPECT_EQ(expected_dir, p.string());
}

TEST(TestFilesystemHelper, parent_absolute_path)
{
  rcpputils::fs::path path("/home/foo/bar/baz");
  if (is_win32) {
    ASSERT_EQ(path.string(), "\\home\\foo\\bar\\baz");
  } else {
    ASSERT_EQ(path.string(), "/home/foo/bar/baz");
  }

  rcpputils::fs::path parent = path.parent_path();
  if (is_win32) {
    ASSERT_EQ(parent.string(), "\\home\\foo\\bar");
  } else {
    ASSERT_EQ(parent.string(), "/home/foo/bar");
  }

  rcpputils::fs::path grandparent = parent.parent_path();
  if (is_win32) {
    ASSERT_EQ(grandparent.string(), "\\home\\foo");
  } else {
    ASSERT_EQ(grandparent.string(), "/home/foo");
  }

  if (is_win32) {
    rcpputils::fs::path win_drive_letter("C:\\home\\foo\\bar");
    ASSERT_EQ(win_drive_letter.string(), "C:\\home\\foo\\bar");
    rcpputils::fs::path win_parent = win_drive_letter.parent_path();
    ASSERT_EQ(win_parent.string(), "C:\\home\\foo");
    rcpputils::fs::path win_grandparent = win_parent.parent_path();
    ASSERT_EQ(win_grandparent.string(), "C:\\home");
  }
}

TEST(TestFilesystemHelper, stream_operator)
{
  path p{"foo"};
  std::stringstream s;
  s << "bar" << p;
  ASSERT_EQ(s.str(), "barfoo");
}

TEST(TestFilesystemHelper, create_temp_directory)
{
  // basic usage
  {
    const std::string basename = "test_base_name";

    const auto tmpdir1 = rcpputils::fs::create_temp_directory(basename);
    EXPECT_TRUE(tmpdir1.exists());
    EXPECT_TRUE(tmpdir1.is_directory());

    auto tmp_file = tmpdir1 / "test_file.txt";
    {
      std::ofstream output_buffer{tmp_file.string()};
      output_buffer << "test";
    }
    EXPECT_TRUE(rcpputils::fs::exists(tmp_file));
    EXPECT_TRUE(rcpputils::fs::is_regular_file(tmp_file));

    const auto tmpdir2 = rcpputils::fs::create_temp_directory(basename);
    EXPECT_TRUE(tmpdir2.exists());
    EXPECT_TRUE(tmpdir2.is_directory());

    EXPECT_NE(tmpdir1.string(), tmpdir2.string());

    EXPECT_TRUE(rcpputils::fs::remove_all(tmpdir1));
    EXPECT_TRUE(rcpputils::fs::remove_all(tmpdir2));
  }

  // bad names
  {
    if (is_win32) {
      EXPECT_THROW(rcpputils::fs::create_temp_directory("illegalchar?"), std::system_error);
    } else {
      EXPECT_THROW(rcpputils::fs::create_temp_directory("base/name"), std::system_error);
    }
  }

  // newly created paths
  {
    const auto new_relative = rcpputils::fs::current_path() / "child1" / "child2";
    const auto tmpdir = rcpputils::fs::create_temp_directory("base_name", new_relative);
    EXPECT_TRUE(tmpdir.exists());
    EXPECT_TRUE(tmpdir.is_directory());
    EXPECT_TRUE(rcpputils::fs::remove_all(tmpdir));
  }

  // edge case inputs
  {
    // Provided no base name we should still get a path with the 6 unique template chars
    const auto tmpdir_emptybase = rcpputils::fs::create_temp_directory("");
    EXPECT_EQ(tmpdir_emptybase.filename().string().size(), 6u);

    // Empty path doesn't exist and cannot be created
    EXPECT_THROW(rcpputils::fs::create_temp_directory("basename", path()), std::system_error);

    // With the template string XXXXXX already in the name, it will still be there, the unique
    // portion is appended to the end.
    const auto tmpdir_template_in_name = rcpputils::fs::create_temp_directory("base_XXXXXX");
    EXPECT_TRUE(tmpdir_template_in_name.exists());
    EXPECT_TRUE(tmpdir_template_in_name.is_directory());
    // On Linux, it will not replace the base_name Xs, only the final 6 that the function appends.
    // On OSX, it will replace _all_ trailing Xs.
    // Either way, the result is unique, the exact value doesn't matter.
    EXPECT_EQ(tmpdir_template_in_name.filename().string().rfind("base_", 0), 0u);
  }
}

TEST(TestFilesystemHelper, equal_operators)
{
  path a{"foo"};
  EXPECT_EQ(a, a);
  path b{"bar"};
  EXPECT_NE(a, b);

  path c = a / b;
  path d = path("foo") / "bar";
  EXPECT_EQ(c, d);
}
