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

#include <yaml.h>

#include <string>

#include "rcpputils/filesystem_helper.hpp"

#include "performance_test_fixture/performance_test_fixture.hpp"

using performance_test_fixture::PerformanceTest;

BENCHMARK_F(PerformanceTest, initialize_delete)(benchmark::State & st)
{
  yaml_parser_t parser;
  for (auto _ : st) {
    yaml_parser_initialize(&parser);
    yaml_parser_delete(&parser);
  }
}

BENCHMARK_F(PerformanceTest, yaml_parser_set_input_string)(benchmark::State & st)
{
  yaml_parser_t parser;
  const char * utf8_sequences = "Hi is \xd0\x9f\xd1\x80\xd0\xb8\xd0\xb2\xd0\xb5\xd1\x82!";
  const char * start = utf8_sequences;
  const char * end = start;
  while (*end != '!') {end++;}
  for (auto _ : st) {
    yaml_parser_initialize(&parser);
    yaml_parser_set_input_string(&parser, (unsigned char *)start, end - start);
    yaml_parser_delete(&parser);
  }
}

BENCHMARK_F(PerformanceTest, yaml_parser_set_input_file)(benchmark::State & st)
{
  std::string path =
    (rcpputils::fs::current_path() / "test" / "benchmark" / "benchmark_params.yaml").string();

  yaml_parser_t parser;
  FILE * pFile;
  reset_heap_counters();
  for (auto _ : st) {
    pFile = fopen(path.c_str(), "r");
    if (NULL == pFile) {
      st.SkipWithError("Error opening the file");
      return;
    }
    yaml_parser_initialize(&parser);
    yaml_parser_set_input_file(&parser, pFile);
    yaml_parser_delete(&parser);
    fclose(pFile);
  }
}

BENCHMARK_F(PerformanceTest, yaml_parser_set_input_file_event)(benchmark::State & st)
{
  std::string path =
    (rcpputils::fs::current_path() / "test" / "benchmark" / "benchmark_params.yaml").string();

  yaml_parser_t parser;
  FILE * pFile;
  yaml_event_t event;
  reset_heap_counters();
  for (auto _ : st) {
    pFile = fopen(path.c_str(), "r");
    if (NULL == pFile) {
      st.SkipWithError("Error opening the file");
      return;
    }
    yaml_parser_initialize(&parser);
    yaml_parser_set_input_file(&parser, pFile);
    yaml_parser_parse(&parser, &event);
    yaml_event_delete(&event);
    yaml_parser_delete(&parser);
    fclose(pFile);
  }
}
