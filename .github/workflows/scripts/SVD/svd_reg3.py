#!/usr/bin/env python3


################################################################################
 #
 # Copyright (C) 2022-2023 Maxim Integrated Products, Inc. All Rights Reserved.
 # (now owned by Analog Devices, Inc.),
 # Copyright (C) 2023-2024 Analog Devices, Inc. All Rights Reserved. This software
 # is proprietary to Analog Devices, Inc. and its licensors.
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
 ###############################################################################

import sys
import re
import xml.etree.ElementTree as ET
import textwrap
import os.path

# Separate add_registers to handle ordering between
#   registers and clusters separately. Cluster registers 
#   and registers in struct should be in order by offset.
def add_register_struct(registers, reg_struct):

  if(len(registers) == 0):
    return reg_struct

  global union
  global struct

  cur_offset = 0;
  last_size = 0;
  rsv_num = 0;
  reg_dim_prev = None
  reg_dim = None
  union = 0
  struct = 0
  union_transition = 0  #monitors back to back union
  cell_1 = 0    # cells to monitor old to new addresses..
  cell_2 = 0    # cell_3 is the newest, cell_1 is the oldest
  cell_3 = 0

  registers = sorted(registers, key=lambda child: int(child.find("addressOffset").text, 16))
  for register in registers:
    reg_name = register.find('name').text
    if "[" in reg_name:
      reg_name = re.sub('\[\%s\]', '', reg_name)
    if(reg_name[-2:-1] == "%"):
      reg_name = reg_name[:2]

    reg_der_name = register.get('derivedFrom')
    reg_der = register
    if reg_der_name != None:
      for _register in registers:
        if _register.find('name').text == reg_der:
          reg_der = _register
          break

    # Get the address offset
    reg_offset = register.find('addressOffset').text

    # Get the size of registers
    reg_size = register.find('size')
    if(reg_size == None):
      reg_size= "32"
    else:
      reg_size = reg_size.text

    # Save the dimension of the previous register for adding reserved space
    reg_dim_prev = reg_dim
    if(reg_dim_prev == None):
      reg_dim_prev = "0"

    # Get the dimension if an array of registers
    reg_dim = register.find('dim')
    if(reg_dim == None):
      reg_dim = "0"
    else:
      reg_dim = reg_dim.text

    # Determine r/w characteristics
    reg_access = register.find('access')
    if (reg_access == None):
      reg_access = "__IO"
    else:
      reg_access = reg_access.text
      if(reg_access == "read-write"):
        reg_access = "__IO"
      elif(reg_access == "write-only"):
        reg_access = "__O"
      elif(reg_access == "read-only"):
        reg_access = "__I"
      elif(reg_access == "writeOnce"):
        reg_access = "__O"
      elif(reg_access == "read-writeOnce"):
        reg_access = "__IO"

    if union != struct:
      # See if this is the last register in the union
      union = 0
      union_transition = 0
      cell_1 = 0
      cell_2 = 0
      cell_3 = 0

      for _register in registers:
      # cell_3 is the newest, cell_1 is the oldest
        cell_1 = cell_2
        cell_2 = cell_3
        cell_3 = int(_register.find('addressOffset').text, 16)


        # Continue through registers with lower addresses
        if int(reg_offset, 16) > int(_register.find('addressOffset').text, 16):
          continue

        if (cell_1 != cell_2) and (cell_2 == cell_3):
          union_transition = 1

        # Continue through this register
        if register.find('name').text == _register.find('name').text:
          continue

        if reg_offset == _register.find('addressOffset').text:
          union = 1
          break
        else:
          break

      if union == 0:
        # Write the end of the union
        reg_struct += "    };\n"

      if union == 1 and union_transition == 1:
        reg_struct += "    };\n"
        reg_struct += "    union {\n"
        union_transition = 0

    last_adjust =(int(last_size)//8)
    reg_adjust = (int(reg_size)//8)

    # Add reserved space if needed
    if(int(reg_offset, 16) > (cur_offset + last_adjust)): #or (int(reg_offset, 16)==0 and per_name != "DMA")):
      if(cur_offset == 0 and per_name == "DMA"):
        print("skip (DMA)")
      else:
        offset = (int(reg_offset, 16) - cur_offset)

        if(reg_dim_prev != "0"):
          offset -= int(reg_dim_prev)*last_adjust
        else:
          offset -= last_adjust

        if(offset != 0):

          if(offset == 1):
            reg_struct += "    __R  uint8_t  rsv_"+str(hex(int(reg_offset, 16)-offset))+";\n"
          elif(offset == 2):
            reg_struct += "    __R  uint16_t rsv_"+str(hex(int(reg_offset, 16)-offset))+";\n"
          elif(offset == 4):
            reg_struct += "    __R  uint32_t rsv_"+str(hex(int(reg_offset, 16)-offset))+";\n"
          elif(offset%4 == 0):
            reg_struct += "    __R  uint32_t rsv_"+str(hex(int(reg_offset, 16)-offset))+"_"+str(hex(int(reg_offset, 16)-1))+"["+str(offset//4)+"];\n"
          elif(offset%2 == 0):
            reg_struct += "    __R  uint16_t rsv_"+str(hex(int(reg_offset, 16)-offset))+"_"+str(hex(int(reg_offset, 16)-1))+"["+str(offset//2)+"];\n"
          else:
            reg_struct += "    __R  uint8_t  rsv_"+str(hex(int(reg_offset, 16)-offset))+"_"+str(hex(int(reg_offset, 16)-1))+"["+str(offset)+"];\n"

          rsv_num += 1

    cur_offset = int(reg_offset,16)#+reg_adjust;
    last_size = reg_size

    # Create union if this register shares an address with another register
    if union == 0 and struct == 0:
      cell_1 = 0
      cell_2 = 0
      cell_3 = 0
      union_transition = 0
      for _register in registers:

        if register.find('name').text == _register.find('name').text:
          continue

        if reg_offset == _register.find('addressOffset').text:
          reg_struct += "    union {\n"
          union = 1
          break

    # Handle SLAVE_MULTI[4] and SLAVE0-3 union/structs
    #   Temporary case for I2C Slave Registers
    elif union == 1 and struct == 1:
      if reg_name == "SLAVE0":
        reg_struct += "        struct {\n"

    # Create the variable
    header_name = register.find('headerStructName')
    if header_name == None:
      reg_var = reg_access+" uint"+reg_size+"_t "+reg_name.lower()
    else:
      reg_var = reg_access+" mxc_"+header_name.text.lower()+"_regs_t "+reg_name.lower()

    if(reg_dim != "0"):
      reg_var +="["+reg_dim+"]"
    reg_var +="; /**< <tt>\\b "+reg_offset+":</tt> "+per_name.upper()+" "+reg_name.upper()+" Register */\n"

    # reg_struct wrap the variable
    reg_var = reg_var.split(' ')

    if header_name == None and union == 0 and struct == 0:
      reg_var = ('    {0[0]:<4} {0[1]:<8} {0[2]:<21} {0[3]} {0[4]} {0[5]} {0[6]} {0[7]} {0[8]} {0[9]}'.format(reg_var))

    elif header_name == None and union == 1 and struct == 0:
      reg_var = ('        {0[0]:<4} {0[1]:<8} {0[2]:<17} {0[3]} {0[4]} {0[5]} {0[6]} {0[7]} {0[8]} {0[9]}'.format(reg_var))

    elif header_name == None and union == 1 and struct == 1:
      reg_var = ('            {0[0]:<4} {0[1]:<8} {0[2]:<13} {0[3]} {0[4]} {0[5]} {0[6]} {0[7]} {0[8]} {0[9]}'.format(reg_var))

    else:
      reg_var = ('    {0[0]:<4} {0[1]:<20} {0[2]:<9} {0[3]} {0[4]} {0[5]} {0[6]} {0[7]} {0[8]} {0[9]}'.format(reg_var))


    # Handle SLAVE_MULTI[4] to SLAVE0-3 union/structs
    #   Temporary case for I2C Slave Registers
    if reg_name == "SLAVE_MULTI" and struct == 0:
      struct = 1

    reg_struct += reg_var

    if reg_name == "SLAVE3":
      reg_struct += "        };\n"
      struct = 0;

  return reg_struct

# Separate add_registers to handling ordering between
#   registers and clusters separately. Cluster registers 
#   and registers masks should not be created in order since 
#   cluster registers and registers offsets could be the same.
#   This causes the cluster and register masks to be intermingled,
#   causing confusion. The cluster register masks section should be first,
#   then the register masks section should follow afterwards.
def add_register_mask(registers, reg_mask):

  if(len(registers) == 0):
    return reg_mask

  registers = sorted(registers, key=lambda child: int(child.find("addressOffset").text, 16))

  for register in registers:
    reg_name = register.find('name').text
    if "[" in reg_name:
      reg_name = re.sub('\[\%s\]', '', reg_name)
    if(reg_name[-2:-1] == "%"):
      reg_name = reg_name[:2]

    # Get the address offset
    reg_offset = register.find('addressOffset').text

    reg_mask_temp = "#define MXC_R_"+per_name.upper()+"_"+reg_name.upper()+" ((uint32_t)"+("0x{:08X}UL)".format(int(reg_offset, 16)))+" /**< Offset from "+per_name.upper()+" Base Address: <tt> 0x"+("{:04X}</tt> */".format(int(reg_offset, 16)))
    reg_mask_temp2 = per_name.upper()+"_"+reg_name.upper()+"\n"
    ugfile.write(reg_mask_temp2)

    #print(reg_mask_temp2)
    reg_mask += ('{0[0]} {0[1]:<34} {0[2]} {0[3]} {0[4]} {0[5]} {0[6]} {0[7]} {0[8]} {0[9]} {0[10]} {0[11]}\n'.format(reg_mask_temp.split(" ")))

  return reg_mask

def add_registers(registers, reg_struct, reg_mask):
  reg_struct = add_register_struct(registers, reg_struct)
  reg_mask = add_register_mask(registers, reg_mask)

  return reg_struct, reg_mask

usage_error =  "Usage svd_reg.py <part.svd> <output_path> <user_guide_output>"
usage_error += "  -r to print register mask"
usage_error += "  -d to print register description"
usage_error += "  -f to print field description"

# Check inputs
if(os.path.isfile(sys.argv[1]) == 0):
  print("SVD file not found")
  print(usage_error)
  sys.exit(0)

if(os.path.exists(sys.argv[2]) == 0):
  print("output path file not found")
  print(usage_error)
  sys.exit(0)
if(os.path.exists(sys.argv[3]) != 0):
  print("output path for user guide file already exists")
  print(usage_error)
  sys.exit(0)

svd=sys.argv[1]
path=sys.argv[2]
ug = sys.argv[3]
ugfile = open(ug, 'w')

print_reg_desc = 0
print_field_desc = 0
print_reg_mask = 0

# Check array for the special arguments
for arg in sys.argv:
  if(arg == "-r"):
    print_reg_mask = 1
  if(arg == "-d"):
    print_reg_desc = 1
  if(arg == "-f"):
    print_field_desc = 1



#print("Creating header files for "+svd)
#print("Output path "+path)
################################################################################
# String placed at the top of each file
copyright = """/******************************************************************************
 *
 * Copyright (C) 2022-2023 Maxim Integrated Products, Inc. All Rights Reserved.
 * (now owned by Analog Devices, Inc.),
 * Copyright (C) 2023-2024 Analog Devices, Inc. All Rights Reserved. This software
 * is proprietary to Analog Devices, Inc. and its licensors.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 ******************************************************************************/
"""

################################################################################
# Create the xml objects
svd_tree = ET.parse(svd)
svd_root = svd_tree.getroot()

# Extract the name and version
device = svd_root.find('name').text.lower()
version = svd_root.find('version').text
print(device)
print("Version "+version)
print("")

# Iterate through each peripheral
################################################################################
peripherals = svd_root.findall('./peripherals/peripheral')

for peripheral in peripherals:
  per_der = peripheral.get('derivedFrom')

  # Skip derived peripherals
  if(per_der != None):
    continue

  # Remove the peripheral number
  per_name = peripheral.find('name').text

  # Add any peripheral with numbers to exclude removing the end digit such as CSI2.
  exclude_periph = ['csi2', 'adc9']
  
  if(per_name[-1:].isdigit()):
    if(per_name.lower().strip() not in exclude_periph):
      per_name = per_name[:-1]

  # Get the description
  per_desc = peripheral.find('description')
  if(per_desc == None):
    per_desc = ""
  else:
    per_desc = per_desc.text

# Create device header file header
  header = """/**
 * @file    """+per_name.lower()+"""_regs.h
 * @brief   Registers, Bit Masks and Bit Positions for the """+per_name.upper()+""" Peripheral Module.
 * @note    This file is @generated.
 * @ingroup """+per_name.lower()+"""_registers
 */

"""
  header_guard_path = """"""

  if ("_REV" in per_name.upper() or device == "private"):
    header_guard_path = """LIBRARIES_PERIPHDRIVERS_SOURCE_"""

    # Remove the _REVn suffix
    peripheral_name_only = per_name.upper().split("_REV")[0] + "_"

    header_guard_path += peripheral_name_only
  else:
    header_guard_path = """LIBRARIES_CMSIS_DEVICE_MAXIM_""" + device.upper() + "_INCLUDE_"

  includes = """
#ifndef """+header_guard_path+per_name.upper()+"""_REGS_H_
#define """+header_guard_path+per_name.upper()+"""_REGS_H_"""
  includes += """

/* **** Includes **** */
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#if defined (__ICCARM__)
  #pragma system_include
#endif

#if defined (__CC_ARM)
  #pragma anon_unions
#endif
/// @cond
/*
    If types are not defined elsewhere (CMSIS) define them here
*/
#ifndef __IO
#define __IO volatile
#endif
#ifndef __I
#define __I  volatile const
#endif
#ifndef __O
#define __O  volatile
#endif
#ifndef __R
#define __R  volatile const
#endif
/// @endcond

/* **** Definitions **** */

/**
 * @ingroup     """+per_name.lower()

  if "aes" in per_name.lower() and per_name.lower() != "aes":
    includes+= """
 * @ingroup     aes"""

  includes+="""
 * @defgroup    """+per_name.lower()+"""_registers """ +per_name.upper()+"""_Registers
 * @brief       Registers, Bit Masks and Bit Positions for the """+per_name.upper()+""" Peripheral Module."""

  if(per_desc != ""):
    includes += "\n * @details     "+per_desc

  includes+= "\n */\n"

  includes+= """
/**
 * @ingroup """+per_name.lower()+"""_registers
 * Structure type to access the """+per_name.upper()+""" Registers.
 */
"""
  # Create register file
  file_path = path+"/"+per_name.lower()+"_regs.h"
 #print("Creating "+file_path)
  register_file = open(file_path, 'w')

  register_file.write(header)
  register_file.write(copyright)
  register_file.write(includes)

################################################################################

  # Create struct for registers
  reg_struct = "typedef struct {\n"
  reg_mask = """
/* Register offsets for module """+per_name.upper()+""" */
/**
 * @ingroup    """+per_name.lower()+"""_registers
 * @defgroup   """+per_name.upper()+"""_Register_Offsets Register Offsets
 * @brief      """+per_name.upper()+""" Peripheral Register Offsets from the """+per_name.upper()+""" Base Peripheral Address.
 * @{
 */
"""

  # Create the clusters
  clusters = peripheral.findall('./registers/cluster')
  for cluster in clusters:
    registers = cluster.findall('./register')
    reg_mask_add = ""
    cluster_struct, reg_mask_add = add_registers(registers, reg_struct, reg_mask_add)
    reg_mask += reg_mask_add
    cluster_name = cluster.find('name').text
    if "[" in per_name: # yank out the %s if this is part of a dim variable
      per_name = re.sub('\[\%[s,S]\]', '', per_name)
    if "[" in cluster_name: # yank out the %s if this is part of a dim variable
      cluster_name = re.sub('\[\%[s,S]\]', '', cluster_name)
    cluster_struct += "} mxc_"+per_name.lower()+"_"+cluster_name.lower()+"_regs_t;\n\n"
    register_file.write(cluster_struct)

  # Create the registers
  registers = peripheral.findall('./registers/register')
  registers += peripheral.findall('./registers/register derivedFrom')

  registers = sorted(registers, key=lambda child: int(child.find("addressOffset").text, 16))

  # Separate ordering of registers and cluster registers
  cluster = peripheral.findall('./registers/cluster')
  cluster += peripheral.findall('./registers/cluster derivedFrom')

  cluster = sorted(cluster, key=lambda child: int(child.find("addressOffset").text, 16))

  registers += cluster

  reg_mask_add = ""
  reg_struct, reg_mask_add = add_registers(registers, reg_struct, reg_mask_add)
  reg_mask += reg_mask_add

  cluster_registers = peripheral.findall('./registers/cluster/register')
  cluster_registers += peripheral.findall('./registers/cluster/register derivedFrom')

  cluster_registers = sorted(cluster_registers, key=lambda child: int(child.find("addressOffset").text, 16))
  registers += cluster_registers

  if struct == 1:
    reg_struct += "        };\n"  # this is for the struct block if the } has not been placed...

  if union == 1:
    reg_struct += "    };\n" # this is for the union block if the } has not been placed...

  if "[" in per_name: # yank out the %s if this is part of a dim variable
      per_name = re.sub('\[\%[s,S]\]', '', per_name)
  reg_struct += "} mxc_"+per_name.lower()+"_regs_t;\n"
  register_file.write(reg_struct)

  if(print_reg_mask):
    register_file.write(reg_mask)
    register_file.write("/**@} end of group "+per_name.lower()+"_registers */\n")


################################################################################
 # Create Macros for each register

  reg_text = ""
  reg_index = 0
  for register in registers:
    reg_name = register.find('name').text.upper()
    if "[" in reg_name:
      reg_name = re.sub('\[\%[s,S]\]', '', reg_name)
    reg_desc = register.find('description').text
    # Get the size of registers
    reg_size = register.find('size')
    if(reg_size == None):
      reg_size= "32"
    else:
      reg_size = reg_size.text
    wut = textwrap.wrap(reg_desc, width = 80)
    reg_desc="\n *           ".join(wut)

    if(register.find('fields') == None):
      continue

    reg_index += 1
    reg_text +="""
/**
"""
    reg_text+=""" * @ingroup  """+per_name.lower()+"""_registers
 * @defgroup """+per_name.upper()+"""_"""+reg_name.upper()+""" """ +per_name.upper()+"""_"""+reg_name.upper() +"""
 * @brief    """+reg_desc+"""
 * @{
 */
"""
    # Create field macros
    fields = register.findall('.fields/field')

    #TODO derived from fields
    for field in fields:
      field_array = []
      field_name = field.find('name').text

      if(field.find('bitOffset') == None):
        if(field.find('lsb') == None):
          if(field.find('bitRange') == None):
            print("ERROR, can't determine field range")
            print("Peripheral: "+per_name)
            print("Register: "+reg_name)
            print("Field: "+field_name)
            sys.exit(1)

          field_lsb = field.find('bitRange').text
          field_lsb = field_lsb.split(':')[1]
          field_lsb = field_lsb[:-1]

        else:
          field_lsb = field.find('lsb').text
      else:
        field_lsb = field.find('bitOffset').text

      if(field.find('bitWidth') == None):
        if(field.find('lsb') == None):
          if(field.find('bitRange') == None):
            print("ERROR, can't determine field range")
            print("Peripheral: "+per_name)
            print("Register: "+reg_name)
            print("Field: "+field_name)
            sys.exit(1)

          field_msb = field.find('bitRange').text
          field_msb = field_msb.split(':')[0]
          field_msb = field_msb[1:]
          field_width = str(int(field_msb) - int(field_lsb) + 1)

        else:
          field_width = str(int(field.find('msb').text) - int(field_lsb) + 1)
      else:
        field_width = field.find('bitWidth').text

      field_desc = field.find('description')

      # Create field description if option selected and available
      if(field_desc != None and print_field_desc):
        field_desc = field_desc.text
        field_desc_array = textwrap.wrap(field_desc, width = 80)
        field_text = " /** \n"
        for line in field_desc_array:
          field_text += "  * "+line +"\n"
        field_text +="  */\n"
        field_array.append(field_text)

      # Create field position
      field_text = "#define MXC_F_"+per_name.upper()+"_"+reg_name.upper()+"_"+field_name.upper()+"_POS "+field_lsb+" /**< "+reg_name.upper()+"_"+field_name.upper()+" Position */\n"
      field_text2 = per_name.upper()+"_"+reg_name.upper()+"."+field_name.lower()+"\n"
      # print(field_text2)
      # print(field_name.lower())
      ugfile.write(field_text2)
      ugfile.write(field_name.lower()+"\n")

      field_text = field_text.split(' ')
      field_array.append('{0[0]} {0[1]:<46} {0[2]} {0[3]} {0[4]} {0[5]} {0[6]}'.format(field_text))

      # Create field mask
      field_mask = 0xFFFFFFFF >> (32 - int(field_width))
      field_text = "#define MXC_F_"+per_name.upper()+"_"+reg_name.upper()+"_"+field_name.upper()+" ((uint"+reg_size+"_t)("+("0x{:X}".format(field_mask))+"UL << MXC_F_"+per_name.upper()+"_"+reg_name.upper()+"_"+field_name.upper()+"_POS))"+" /**< "+reg_name.upper()+"_"+field_name.upper()+" Mask */\n"
      field_text = field_text.split(' ')
      field_array.append('{0[0]} {0[1]:<46} {0[2]} {0[3]} {0[4]} {0[5]} {0[6]} {0[7]} {0[8]}'.format(field_text))

      # Create macro for enumerated values
      enums_set = field.findall('.enumeratedValues')

      # TODO: Get from "derivedFrom"

      # BLE Spec (DBB registers) should have their enums printed out.
      if ((int(field_width) > 1) or (device == "max32650") or (device == "dbb")):
        for enums in enums_set:
          enums_ = enums.findall('.enumeratedValue')
          for enum in enums_:
            enum_name = enum.find('name').text
            enum_value = enum.find('value')
            enum_desc = enum.find('description')
            if(enum_desc != None):
              enum_desc = enum_desc.text

            if(enum_value == None):
              continue

            # Convert enum value to hex
            if(enum_value.text[:2] == "0x" or enum_value.text[:2] == "0X"):
              enum_value = int(enum_value.text, 16)
            else:
              enum_value = int(enum_value.text, 10)

            field_text = "#define MXC_V_"+per_name.upper()+"_"+reg_name.upper()+"_"+field_name.upper()+"_"+enum_name.upper()+ " ((uint"+ reg_size +"_t)"+("0x{:X}".format(enum_value))+"UL)"+" /**< "+reg_name.upper()+"_"+field_name.upper()+"_"+enum_name.upper()+" Value */"
            field_text = field_text.split(' ')
            field_array.append('{0[0]} {0[1]:<46} {0[2]} {0[3]} {0[4]} {0[5]} {0[6]}\n'.format(field_text))

            field_text = "#define MXC_S_"+per_name.upper()+"_"+reg_name.upper()+"_"+field_name.upper()+"_"+enum_name.upper()+" (MXC_V_"+per_name.upper()+"_"+reg_name.upper()+"_"+field_name.upper()+"_"+enum_name.upper()+" << "+"MXC_F_"+per_name.upper()+"_"+reg_name.upper()+"_"+field_name.upper()+"_POS)"+" /**< "+reg_name.upper()+"_"+field_name.upper()+"_"+enum_name.upper()+" Setting */"
            field_text = field_text.split(' ')
            field_array.append('{0[0]} {0[1]:<46} {0[2]} {0[3]} {0[4]} {0[5]} {0[6]} {0[7]} {0[8]}\n'.format(field_text))

      field_array.append("\n")

      for field_line in field_array:
        reg_text+=field_line

    reg_text += "/**@} end of group "+per_name.upper()+"_"+reg_name.upper()+"_Register */\n"

  register_file.write(reg_text)


################################################################################
  footer="""
#ifdef __cplusplus
}
#endif

#endif // """+header_guard_path+per_name.upper()+"""_REGS_H_
"""
  register_file.write(footer)
  register_file.close()
