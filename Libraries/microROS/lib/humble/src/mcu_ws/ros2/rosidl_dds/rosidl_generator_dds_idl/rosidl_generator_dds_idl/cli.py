# Copyright 2021 Open Source Robotics Foundation, Inc.
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

import pathlib

from ament_index_python import get_package_share_directory
from rosidl_cli.command.helpers import legacy_generator_arguments_file
from rosidl_cli.command.translate.extensions import TranslateCommandExtension

from rosidl_generator_dds_idl import generate_dds_idl


class TranslateIDLToDDSIDL(TranslateCommandExtension):

    input_format = 'idl'
    output_format = 'dds_idl'

    def translate(
        self,
        package_name,
        interface_files,
        include_paths,
        output_path
    ):
        package_share_path = pathlib.Path(
            get_package_share_directory('rosidl_generator_dds_idl'))
        templates_path = package_share_path / 'resource'

        with legacy_generator_arguments_file(
            package_name=package_name,
            interface_files=interface_files,
            include_paths=include_paths,
            templates_path=templates_path,
            output_path=output_path
        ) as path_to_arguments_file:
            generated_files = generate_dds_idl(
                path_to_arguments_file,
                subfolders=[],
                extension_module_name=None,
                additional_service_templates=[]
            )
        translated_interface_files = []
        for path in generated_files:
            path = pathlib.Path(path)
            relative_path = path.relative_to(output_path)
            translated_interface_files.append(
                f'{output_path}:{relative_path.as_posix()}'
            )
        return translated_interface_files
