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

import os

from rosidl_cmake import generate_files
from rosidl_parser.definition import AbstractGenericString
from rosidl_parser.definition import AbstractNestableType
from rosidl_parser.definition import AbstractString
from rosidl_parser.definition import AbstractWString
from rosidl_parser.definition import BasicType
from rosidl_parser.definition import FLOATING_POINT_TYPES
from rosidl_parser.definition import NamespacedType


def generate_dds_idl(
    generator_arguments_file, subfolders, extension_module_name,
    additional_service_templates
):
    mapping = {
        'idl.idl.em': os.path.join(*subfolders, '%s_.idl')
    }
    additional_context = {
      'additional_service_templates': additional_service_templates,
      'subfolders': subfolders,
      'get_post_struct_lines': get_post_struct_lines,
      'idl_typename': idl_typename,
      'idl_literal': idl_literal,
    }
    # Look for extensions for additional context
    if extension_module_name is not None:
        pkg = __import__(extension_module_name)
        module_name = extension_module_name.rsplit('.', 1)[1]
        if hasattr(pkg, module_name):
            module = getattr(pkg, module_name)
            for entity_name in additional_context.keys():
                if hasattr(module, entity_name):
                    additional_context[entity_name] = \
                        getattr(module, entity_name)
    return generate_files(generator_arguments_file, mapping, additional_context, keep_case=True)


# used by the template
def get_post_struct_lines(message):
    return []


# used by the template
EXPLICIT_TYPE_TO_IMPLICIT_TYPE = {
    'int8': 'octet',
    'uint8': 'octet',
    'int16': 'short',
    'uint16': 'unsigned short',
    'int32': 'long',
    'uint32': 'unsigned long',
    'int64': 'long long',
    'uint64': 'unsigned long long',
}


# used by the template
def idl_typename(type_):
    assert(isinstance(type_, AbstractNestableType))
    if isinstance(type_, BasicType):
        typename = EXPLICIT_TYPE_TO_IMPLICIT_TYPE.get(type_.typename, type_.typename)
    elif isinstance(type_, AbstractGenericString):
        if isinstance(type_, AbstractString):
            typename = 'string'
        elif isinstance(type_, AbstractWString):
            typename = 'wstring'
        else:
            assert False, 'Unknown string type'
        if type_.has_maximum_size():
            typename += '<%d>' % (type_.maximum_size)
    elif isinstance(type_, NamespacedType):
        typename = '::'.join(type_.namespaces + ['dds_', type_.name + '_'])
    else:
        assert False, 'Unknown base type'
    return typename


# used by the template
def idl_literal(type_, value):
    assert(isinstance(type_, AbstractNestableType))
    if isinstance(type_, BasicType):
        if type_.typename == 'boolean':
            literal = 'TRUE' if value else 'FALSE'
        elif type_.typename == 'char':
            literal = '%s' % value
        elif type_.typename == 'int8':
            literal = '%d' % (value if value >= 0 else value + 256)
        elif type_.typename in FLOATING_POINT_TYPES:
            literal = '%f' % value
        else:
            literal = '%d' % value
    elif isinstance(type_, AbstractString):
        literal = '"%s"' % value
    elif isinstance(type_, AbstractWString):
        literal = 'L"%s"' % value
    else:
        assert False, 'Unknown base type'
    return literal
