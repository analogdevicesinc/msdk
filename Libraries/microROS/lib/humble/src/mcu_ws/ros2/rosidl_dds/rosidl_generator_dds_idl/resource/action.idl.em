@# Included from rosidl_generator_dds_idl/resource/idl.idl.em
@{
TEMPLATE(
    'msg.idl.em',
    package_name=package_name,
    interface_path=interface_path,
    message=action.goal,
    get_post_struct_lines=get_post_struct_lines,
    idl_typename=idl_typename, idl_literal=idl_literal,
)
TEMPLATE(
    'srv.idl.em',
    package_name=package_name,
    interface_path=interface_path,
    service=action.send_goal_service,
    get_post_struct_lines=get_post_struct_lines,
    idl_typename=idl_typename, idl_literal=idl_literal,
    additional_service_templates=additional_service_templates,
)
TEMPLATE(
    'msg.idl.em',
    package_name=package_name,
    interface_path=interface_path,
    message=action.result,
    get_post_struct_lines=get_post_struct_lines,
    idl_typename=idl_typename, idl_literal=idl_literal,
)
TEMPLATE(
    'srv.idl.em',
    package_name=package_name,
    interface_path=interface_path,
    service=action.get_result_service,
    get_post_struct_lines=get_post_struct_lines,
    idl_typename=idl_typename, idl_literal=idl_literal,
    additional_service_templates=additional_service_templates,
)
TEMPLATE(
    'msg.idl.em',
    package_name=package_name,
    interface_path=interface_path,
    message=action.feedback,
    get_post_struct_lines=get_post_struct_lines,
    idl_typename=idl_typename, idl_literal=idl_literal,
)
TEMPLATE(
    'msg.idl.em',
    package_name=package_name,
    interface_path=interface_path,
    message=action.feedback_message,
    get_post_struct_lines=get_post_struct_lines,
    idl_typename=idl_typename, idl_literal=idl_literal,
)
}@
