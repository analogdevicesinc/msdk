// generated from rosidl_generator_dds_idl/resource/idl.idl.em
// with input from @(package_name):@(interface_path.as_posix())
// generated code does not contain a copyright notice
@{
import os
from rosidl_parser.definition import Include
includes = content.get_elements_of_type(Include)
}@
@[if includes]@
@{
include_directives = set()
}@
@[  for include in includes]@
@{
name, ext = os.path.splitext(include.locator)
dir_name = os.path.dirname(name)
include_name = '{}_{}'.format('/'.join([dir_name] + subfolders + [os.path.basename(name)]), ext)
}@
@[    if include_name not in include_directives]@
#include "@(include_name)"
@[    end if]@
@{
include_directives.add(include_name)
}@
@[  end for]@
@[end if]@
@{
from rosidl_parser.definition import Action
from rosidl_parser.definition import Message
from rosidl_parser.definition import Service

from rosidl_cmake import convert_camel_case_to_lower_case_underscore
header_guard_parts = [package_name] + list(interface_path.parents[0].parts) + \
    [convert_camel_case_to_lower_case_underscore(interface_path.stem)] + \
    ['idl']
}@

#ifndef __@('__'.join(header_guard_parts))__
#define __@('__'.join(header_guard_parts))__

@{
for message in content.get_elements_of_type(Message):
    TEMPLATE(
        'msg.idl.em', package_name=package_name,
        interface_path=interface_path, message=message,
        get_post_struct_lines=get_post_struct_lines,
        idl_typename=idl_typename, idl_literal=idl_literal,
    )

for service in content.get_elements_of_type(Service):
    TEMPLATE(
        'srv.idl.em', package_name=package_name,
        interface_path=interface_path, service=service,
        get_post_struct_lines=get_post_struct_lines,
        idl_typename=idl_typename, idl_literal=idl_literal,
        additional_service_templates=additional_service_templates,
    )

for action in content.get_elements_of_type(Action):
    TEMPLATE(
        'action.idl.em', package_name=package_name,
        interface_path=interface_path, action=action,
        get_post_struct_lines=get_post_struct_lines,
        idl_typename=idl_typename, idl_literal=idl_literal,
        additional_service_templates=additional_service_templates,
    )
}@

#endif  // __@('__'.join(header_guard_parts))__
