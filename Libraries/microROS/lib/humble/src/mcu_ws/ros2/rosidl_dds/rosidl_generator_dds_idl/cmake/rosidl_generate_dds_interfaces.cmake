# Copyright 2014-2015 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

#
# Generate DDS IDL files from ROS IDL files.
#
# :param target: the name of the generation target,
# :type target: string
# :param IDL_TUPLES: a list of IDL files of the form
#   "path/to:subfolder/Message.idl"
# :type IDL_TUPLES: list of tuples of colon separated strings
# :param DEPENDENCY_PACKAGE_NAMES: a list of dependency package names
# :type DEPENDENCY_PACKAGE_NAMES: list of strings
# :param SERVICE_TEMPLATES: a list of additional EmPy templates to evaluate
#   when generating services
# :type SERVICE_TEMPLATES: list of strings
# :param OUTPUT_SUBFOLDERS: a list of subfolders between the package name and
#   the interface name
# :type OUTPUT_SUBFOLDERS: optional list of strings
# :param EXTENSION: a Python module extending the generator
# :type EXTENSION: optional string
#
# @public
#
macro(rosidl_generate_dds_interfaces target)
  cmake_parse_arguments(_ARG "" "EXTENSION"
    "IDL_TUPLES;DEPENDENCY_PACKAGE_NAMES;OUTPUT_SUBFOLDERS;SERVICE_TEMPLATES" ${ARGN})
  if(_ARG_UNPARSED_ARGUMENTS)
    message(FATAL_ERROR "rosidl_generate_dds_interfaces() called with "
      "unused arguments: ${_ARG_UNPARSED_ARGUMENTS}")
  endif()

  set(_output_basepath "${CMAKE_CURRENT_BINARY_DIR}/rosidl_generator_dds_idl/${PROJECT_NAME}")
  set(_abs_idl_files "")
  set(_generated_files "")
  set(_generated_dirs "")
  foreach(_idl_tuple ${_ARG_IDL_TUPLES})
    string(REGEX REPLACE ":([^:]*)$" "/\\1" _abs_idl_file "${_idl_tuple}")
    list(APPEND _abs_idl_files "${_abs_idl_file}")
    get_filename_component(_parent_folder "${_abs_idl_file}" DIRECTORY)
    get_filename_component(_parent_folder "${_parent_folder}" NAME)
    get_filename_component(_name "${_abs_idl_file}" NAME_WE)
    set(_output_dirpath "${_parent_folder}")
    foreach(_subfolder ${_ARG_OUTPUT_SUBFOLDERS})
      set(_output_dirpath "${_output_dirpath}/${_subfolder}")
    endforeach()
    list(APPEND _generated_dirs "${_output_dirpath}")
    set(_output_path "${_output_basepath}/${_output_dirpath}")
    list(APPEND _generated_files "${_output_path}/${_name}_.idl")
  endforeach()
  list(REMOVE_DUPLICATES _generated_dirs)

  set(_dependency_files "")
  set(_dependencies "")
  foreach(_pkg_name ${rosidl_generate_interfaces_DEPENDENCY_PACKAGE_NAMES})
    foreach(_idl_file ${${_pkg_name}_IDL_FILES})
      set(_abs_idl_file "${${_pkg_name}_DIR}/../${_idl_file}")
      normalize_path(_abs_idl_file "${_abs_idl_file}")
      list(APPEND _dependency_files "${_abs_idl_file}")
      list(APPEND _dependencies "${_pkg_name}:${_abs_idl_file}")
    endforeach()
  endforeach()

  set(target_dependencies
    "${rosidl_generator_dds_idl_BIN}"
    ${rosidl_generator_dds_idl_GENERATOR_FILES}
    "${rosidl_generator_dds_idl_TEMPLATE_DIR}/action.idl.em"
    "${rosidl_generator_dds_idl_TEMPLATE_DIR}/idl.idl.em"
    "${rosidl_generator_dds_idl_TEMPLATE_DIR}/msg.idl.em"
    "${rosidl_generator_dds_idl_TEMPLATE_DIR}/srv.idl.em"
    ${_ARG_SERVICE_TEMPLATES}
    ${_abs_idl_files}
    ${_dependency_files})
  foreach(dep ${target_dependencies})
    if(NOT EXISTS "${dep}")
      get_property(is_generated SOURCE "${dep}" PROPERTY GENERATED)
      if(NOT ${_is_generated})
        message(FATAL_ERROR "Target dependency '${dep}' does not exist")
      endif()
    endif()
  endforeach()

  # use unique arguments file for each subfolder
  set(generator_arguments_file "${CMAKE_CURRENT_BINARY_DIR}/rosidl_generator_dds_idl_")
  foreach(_subfolder ${_ARG_OUTPUT_SUBFOLDERS})
    set(generator_arguments_file "${generator_arguments_file}_${_subfolder}_")
  endforeach()
  set(generator_arguments_file "${generator_arguments_file}_arguments.json")
  rosidl_write_generator_arguments(
    "${generator_arguments_file}"
    PACKAGE_NAME "${PROJECT_NAME}"
    IDL_TUPLES "${_ARG_IDL_TUPLES}"
    ROS_INTERFACE_DEPENDENCIES "${_dependencies}"
    OUTPUT_DIR "${_output_basepath}"
    TEMPLATE_DIR "${rosidl_generator_dds_idl_TEMPLATE_DIR}"
    TARGET_DEPENDENCIES ${target_dependencies}
  )

  add_custom_command(
    OUTPUT ${_generated_files}
    COMMAND ${PYTHON_EXECUTABLE} ${rosidl_generator_dds_idl_BIN}
    --additional-service-templates ${_ARG_SERVICE_TEMPLATES}
    --generator-arguments-file "${generator_arguments_file}"
    --subfolders ${_ARG_OUTPUT_SUBFOLDERS}
    --extension ${_ARG_EXTENSION}
    DEPENDS ${target_dependencies}
    COMMENT "Generating DDS interfaces"
    VERBATIM
  )

  add_custom_target(
    ${target}
    DEPENDS
    ${_generated_files}
  )

  set(_idl_destination "share/${PROJECT_NAME}")
  if(NOT rosidl_generate_interfaces_SKIP_INSTALL)
    if(NOT _generated_dirs STREQUAL "")
      foreach(_dir ${_generated_dirs})
        install(
          DIRECTORY "${_output_basepath}/${_dir}/"
          DESTINATION "${_idl_destination}/${_dir}"
        )
      endforeach()
    endif()
  endif()
endmacro()
