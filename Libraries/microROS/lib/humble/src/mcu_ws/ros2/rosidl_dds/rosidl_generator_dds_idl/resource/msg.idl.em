@# Included from rosidl_generator_dds_idl/resource/idl.idl.em
@{
from rosidl_parser.definition import AbstractNestableType
from rosidl_parser.definition import AbstractNestedType
from rosidl_parser.definition import AbstractSequence
from rosidl_parser.definition import Array
from rosidl_parser.definition import BoundedSequence
from rosidl_parser.definition import CONSTANT_MODULE_SUFFIX

}@

@[for ns in message.structure.namespaced_type.namespaces]@
module @(ns) {

@[end for]@
module dds_ {

@[if message.constants]@
module @(message.structure.namespaced_type.name)@(CONSTANT_MODULE_SUFFIX) {
@[  for constant in message.constants]@
const @(idl_typename(constant.type)) @(constant.name)_ = @(idl_literal(constant.type, constant.value));
@[  end for]
};
@[end if]@

struct @(message.structure.namespaced_type.name)_ {
@[for member in message.structure.members]@
@[  for value in member.get_annotation_values('key')]@
@@key@
@[    if value]@
("@(value)")@
@[    end if]
@[  end for]@
@[  if isinstance(member.type, AbstractNestedType)]@
@[    if isinstance(member.type, Array)]@
@(idl_typename(member.type.value_type)) @(member.name)_[@(member.type.size)];
@[    elif isinstance(member.type, AbstractSequence)]@
sequence<@(idl_typename(member.type.value_type))@
@[      if isinstance(member.type, BoundedSequence)]@
, @(member.type.maximum_size)@
@[      elif idl_typename(member.type.value_type).endswith('>')]@
 @
@[      end if]@
> @(member.name)_;
@[    else]@

@[    end if]
@[  elif isinstance(member.type, AbstractNestableType)]@
@(idl_typename(member.type)) @(member.name)_;
@[  else]@

@[  end if]@
@[end for]@

};

@[for line in get_post_struct_lines(message)]@
@(line)
@[end for]@

};  // module dds_

@[for ns in reversed(message.structure.namespaced_type.namespaces)]@
};  // module @(ns)

@[end for]@
