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

#ifdef _WIN32
#define _CRT_SECURE_NO_WARNINGS
#endif


#include <gtest/gtest.h>
#include <yaml.h>

#include <string>

#include "rcpputils/filesystem_helper.hpp"
#include "rcutils/filesystem.h"

TEST(test_libyaml, yaml_read_file)
{
  std::string path =
    (rcpputils::fs::current_path() / "test" / "benchmark" / "benchmark_params.yaml").string();

  yaml_parser_t parser;
  yaml_event_t event;
  FILE * pFile;
  pFile = fopen(path.c_str(), "r");
  EXPECT_NE(pFile, nullptr);
  EXPECT_EQ(1, yaml_parser_initialize(&parser));
  yaml_parser_set_input_file(&parser, pFile);
  EXPECT_EQ(1, yaml_parser_parse(&parser, &event));
  yaml_event_delete(&event);
  yaml_parser_delete(&parser);
  fclose(pFile);
}

TEST(test_libyaml, yaml_read_file_fails)
{
  std::string path =
    (rcpputils::fs::current_path() / "test" / "benchmark" / "benchmark_params.yaml").string();

  yaml_parser_t parser;
  yaml_event_t event;
  FILE * pFile;
  pFile = fopen(path.c_str(), "r");
  EXPECT_NE(pFile, nullptr);

  EXPECT_DEATH_IF_SUPPORTED(yaml_parser_initialize(nullptr), "");

  EXPECT_DEATH_IF_SUPPORTED(yaml_parser_parse(&parser, nullptr), "");
  EXPECT_DEATH_IF_SUPPORTED(yaml_parser_parse(nullptr, &event), "");

  EXPECT_DEATH_IF_SUPPORTED(yaml_event_delete(nullptr), "");
  EXPECT_DEATH_IF_SUPPORTED(yaml_parser_delete(nullptr), "");
}
