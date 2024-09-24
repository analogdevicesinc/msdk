###############################################################################
 #
 # Copyright (C) 2024 Analog Devices, Inc.
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
 ##############################################################################


# This script 1) allows the user to select their memory settings for their
#   Secure & Non-Secure projects (TrustZone) in their project.mk file,
#   and 2) it will generate the appropriate linker scripts and 
#   partion_{device}.h file into their project directory.
#
# This script is not run by the users, but a part of the MSDK build system.

# NOTE: This script was written with minimal dependencies to libraries that
#   customers have to install. If future support is added, try not to add
#   libraries that are not default to a simple python install. Users should
#   expect to install the MSDK and not have to install any additional
#   components.
#

import sys
import argparse
import re
import shutil
import os


# Notes for future maintainers:
#   1. Each print statement starts with "> " to help format the output
#       in the build system {device}_memory.mk

###############################################################################
#
#   SDK MAINTAINER MUST EDIT THIS SECTION FOR EACH PART.
#
###############################################################################
#
# Setup device's default physical memory definitons.
#
# Note for the MAX32657 (ME30), bit 28 of an address indicates the security 
#   status. Clear this bit to get the physical address. Use the virtual
#   address during development.
#       address[28] = 1 -Secure
#       address[28] = 0 -Non-Secure
#
# Too many definitions to make th
#
################################################################################

TARGET = "MAX32657"

# Flash Settings.
PHY_FLASH_INST_NUM = 1

# Total Flash.
PHY_FLASH_BASE = 0x01000000
PHY_FLASH_SIZE = 0x00100000

# Individual Flash.
#   Adding to support future parts that could potentially have more than
#   one Flash region.
PHY_FLASH_BASE_INST = [
    0x01000000 # Flash 0
]

PHY_FLASH_SIZE_INST = [
    0x00100000 # Flash 0 - 1MiB
]

# Memory Protection Controller (MPC) Flash Block Size.
#   This variable is setup this way for future compatibility
#   because each Memory Region has its own MPC, and each
#   MPC can potentially have varying block sizes (though
#   unlikely).
MPC_FLASH_BLOCK_SIZE = [
    0x00008000 # MPC Flash 0 Block Size (32KiB)
]

# SRAM Settings.
PHY_SRAM_INST_NUM = 5

# Total SRAM.
PHY_SRAM_BASE = 0x20000000
PHY_SRAM_SIZE = 0x00040000

# Individual SRAM.
PHY_SRAM_BASE_INST = [
    0x20000000, # SRAM0
    0x20008000, # SRAM1
    0x20010000, # SRAM2
    0x20020000, # SRAM3
    0x20030000  # SRAM4
]

PHY_SRAM_SIZE_INST = [
    0x00008000, # SRAM0 - 32KiB
    0x00008000, # SRAM1 - 32KiB
    0x00010000, # SRAM2 - 64KiB
    0x00010000, # SRAM3 - 64KiB
    0x00010000  # SRAM4 - 64KiB
]

# Memory Protection Controller (MPC) SRAM Block Size.
#   This variable is setup this way for future compatibility
#   because each Memory Region has its own MPC, and each
#   MPC can potentially have varying block sizes (though
#   unlikely).
#   At the time this was written (MAX32657), the SRAM MPCs all
#   had the same block sizes.
MPC_SRAM_BLOCK_SIZE = [
    0x00001000, # MPC SRAM0 Block Size (4KiB)
    0x00001000, # MPC SRAM1 Block Size (4KiB)
    0x00001000, # MPC SRAM2 Block Size (4KiB)
    0x00001000, # MPC SRAM3 Block Size (4KiB)
    0x00001000  # MPC SRAM4 Block Size (4KiB)
]

# Location of template linker scripts from MAXIM_PATH environment variable.
SECURE_PARTITION_TEMPLATE_PATH = f"Libraries/CMSIS/Device/Maxim/{TARGET.upper()}/Source/Template/partition_max32657.h"
SECURE_LINKER_TEMPLATE_PATH = f"Libraries/CMSIS/Device/Maxim/{TARGET.upper()}/Source/Template/max32657_s.ld"
NONSECURE_LINKER_TEMPLATE_PATH = f"Libraries/CMSIS/Device/Maxim/{TARGET.upper()}/Source/Template/max32657_ns.ld"

###############################################################################

# Dictionary for conversions to bytes. Use lower case to make
#   parsing easier since we're using binary prefixes.
#   SI prefixes treated as binary prefixes.
units_byte_conversions = {
    'b': 1,
    'kb': 1024,
    'kib': 1024,
    'mb': 1024*1024,
    'mib': 1024*1024,
    # Add more if needed.
}

def string_to_integer_bytes(string_hex_or_units):
    try:
        number_bytes = int(string_hex_or_units, 16)
        return number_bytes
    except ValueError:
        # Consistent with conversion dictionary.
        string_units = string_hex_or_units.lower()

        # Shouldn't happen, but remove spaces just in case since
        #   string should be a number with units (32KB, 64B, 1MB).
        string_units = string_units.replace(" ", "")

        # Find location of last number before units begin.
        for i, char in enumerate(string_units):
            if not char.isdigit():
                break

        number = int(string_units[:i])
        units = string_units[i:]

        # Defaults conversion factor to bytes if unit was not specified.
        byte_conversion_factor = units_byte_conversions.get(units, 1)

        number_bytes = number * byte_conversion_factor

        return number_bytes

def subtract_kib_string_units(big_int, small_int):
    # Shouldn't happen, but remove spaces just in case since
    #   string should be a number with units (32KB, 64B, 1MB).
    big_int = big_int.replace(" ", "")
    small_int = small_int.replace(" ", "")

    # Find location of last number before units begin.
    for i, char in enumerate(big_int):
        if not char.isdigit():
            break

    for j, char in enumerate(small_int):
        if not char.isdigit():
            break

    big = int(big_int[:i])
    big_units = big_int[i:]

    small = int(small_int[:j])
    small_units = small_int[j:]

    if big_units != small_units:
        return "Subtraction Error"
    
    result = big - small

    result = str(result) + big_units

    return result

def remove_decimal_0_from_number_string(number: str):
    if "." in number:
        return number.rstrip('0').rstrip('.')
    else:
        # do nothing
        return number

def integer_bytes_to_string_KiB(integer_bytes):
    integer_bytes /= units_byte_conversions.get('kib')

    string_KiB = remove_decimal_0_from_number_string(str(integer_bytes)) + "KiB"

    return string_KiB

###############################################################################

class MemorySettings:
    # Constructor.
    def __init__(self, type: str,    # Type of memory. Two string options: FLASH or SRAM)
                 num_instances: int, # Number of memory instances.
                 starts: list, sizes: list, # For array of individual memory instance settings.
                 mpc_blk_sizes: list,  # block sizes for each MPC that is paired with each instance.
                 execute_code_here: bool # Execute code from this memory type.
                ):
        self.type = type.upper()
        self.instances = num_instances
        self.execute_code_here = execute_code_here

        # Size of entire physical memory region.
        self.phy_size = 0x00000000

        # Ensure physical addresses are used by clearing bit 28 (security state alias
        #   bit) and get total physical size.
        for i in range(num_instances):
            starts[i] &= ~(1<<28)
            self.phy_size += sizes[i]

        # Verify user-configure settings match physical memory constraints above.
        if type.upper() == "FLASH":
            if self.phy_size != PHY_FLASH_SIZE:
                print(f"> [ERROR]: Memory Regions in FLASH must total to '{PHY_FLASH_SIZE}'")
                sys.exit(1)
        if type.upper() == "SRAM":
            if self.phy_size != PHY_SRAM_SIZE:
                print(f"> [ERROR]: Memory Regions in SRAM must total to '{PHY_SRAM_SIZE}'")
                sys.exit(1)

        # Starting address of entire physical memory region.
        #    Region 0 is first instance, so use that for starting
        #    address of physical memory.
        self.phy_start = starts[0]

        # Attributes for phyiscal memory settings.
        self.phy_inst_starts = starts
        self.phy_inst_sizes = sizes
        self.mpc_inst_blk_sizes = mpc_blk_sizes

        # Attributes for Project-defined Memory Regions (arguments of this script).
        self.are_regions_set = False
        self.s_start = None # Secure Region.
        self.s_size = None
        self.ns_start = None # Non-Secure Region.
        self.ns_size = None
        self.nsc_size = None # Non-Secure Callable Region.
        self.nsc_region_present = False

    def __get_mem_inst(self, address):
        for inst in range(self.instances):
            if address in range(self.phy_inst_starts[inst], self.phy_inst_starts[inst] + self.phy_inst_sizes[inst]):
                return inst

    # Only for private use, checks region is aligned to MPC block size.
    #   Doesn't do anything if regions aren't set (setRegions method).
    # This method checks whether the "Non-Secure" region shares the same block as the
    #   "Secure/Non-Secure Callable" regions. The "Secure/Non-Secure Callable" regions
    #   are consided secure type for the MPC.
    #
    # Return: True if everything is aligned, False if not.
    def __check_region_alignment_to_mpc(self):
        if self.are_regions_set:
            starts = []
            sizes = []

            # Use physical addressing for arithmetic.
            s_start = self.s_start & ~(1<<28)
            ns_start = self.ns_start & ~(1<<28)

            starts.append(s_start)
            sizes.append(self.total_s_nsc_size) # S and NSC regions are combined (if NSC region exists in this memory type).
            starts.append(ns_start)
            sizes.append(self.ns_size)

            mpc_regions = {starts[i]: sizes[i] for i in range(len(starts))}

            # For easier allocation, order the regions from least to greatest starting addresses.
            #   Note, by nature, dictionaries are not sorted.
            sorted_mpc_regions = dict(sorted(mpc_regions.items()))

            # First region is always expected to start at beginning of physical memory due to constructor
            #   checking if all of memory was used.
            # For loop and logic below exists for future parts in case more than 2 SAU regions are available
            #   in this memory type.
            for curr_start, curr_size in sorted_mpc_regions.items():
                # Don't go to first element since we're comparing the current and previous region.
                if curr_start != list(sorted_mpc_regions.keys())[0]:
                    # Make sure the curr and prev regions aren't secure and non-secure callable regions since
                    #   to the MPC, they're both considered as secure and can be ignored with this check.
                    if self.is_nonsecurecall_present:
                        # If current and previous region is either non-secure, then do the MPC block check.
                        if prev_start == ns_start or curr_start == ns_start:
                            # Save the border between secure and nonsecure regions.
                            prev_region_end_mem_inst = self.__get_mem_inst(prev_start + prev_size - 1) # -1 for index starting 0.
                            curr_region_start_mem_inst = self.__get_mem_inst(curr_start)
                        else:
                            # Save current region before proceeding to next iteration.
                            prev_start = curr_start
                            prev_size = prev_size
                            continue

                    else: # Only two possible regions: NS and S, so a check must be done.
                        prev_region_end_mem_inst = self.__get_mem_inst(prev_start + prev_size - 1) # -1 for index starting 0.
                        curr_region_start_mem_inst = self.__get_mem_inst(curr_start)

                    # If it reaches here, then there is a Non-Secure/Secure boundary that must be checked.
                    #   Region 0 and Region 1 share the same memory instance. There's a chance they could share
                    #   the same block which is not possible with the granularity of the MPC blocks.
                    if prev_region_end_mem_inst == curr_region_start_mem_inst:
                        # Check MPC alignment.
                        block, remainder = divmod(self.phy_inst_sizes[curr_region_start_mem_inst] - curr_start, self.mpc_inst_blk_sizes[curr_region_start_mem_inst])

                        if remainder != 0:
                            MPC_BLK_SIZE = integer_bytes_to_string_KiB(self.mpc_inst_blk_sizes[curr_region_start_mem_inst])
                            print(f"> [ERROR]: Incorrect '{self.type}' Memory Region Alignment. The memory regions must be aligned with the corresponding '{self.type}' Memory Protection Controller (MPC) Block Size: '{MPC_BLK_SIZE}'")
                            sys.exit(1)

                # Save current region before proceeding to next iteration.
                prev_start = curr_start
                prev_size = curr_size

    # Set up Security regions.
    def setRegions(self, secure_start, secure_size,
                    nonsecure_start, nonsecure_size,
                    nonsecurecall_start, nonsecurecall_size, is_nonsecurecall_present):

        # Constructor only checks for physical memory constraints.
        #   This checks for user configured memory settings.
        starting_region_start = min(secure_start & ~(1<<28), nonsecure_start)
        if (starting_region_start) != self.phy_start:
            # Save bit 28 state for error message
            if (secure_start & ~(1<<28)) == starting_region_start:
                bit28_security = (1 << 28)
            else:
                bit28_security = 0

            print(f"> [ERROR]: Lowest Memory Region does not start at the beginning of '{self.type}': '{hex(self.phy_start | bit28_security)}'")
            sys.exit(1)

        self.s_start = secure_start
        self.s_size = secure_size
        self.ns_start = nonsecure_start
        self.ns_size = nonsecure_size

        self.are_regions_set = True
        self.is_nonsecurecall_present = is_nonsecurecall_present

        # The NSC region is a subset of the Secure region. Treat it holistically - if NSC is present in this memory type.
        self.total_s_nsc_size = secure_size

        # Leave as default (None) set by constructor if NSC region not present in this memory type.
        if is_nonsecurecall_present:
            self.nsc_start = nonsecurecall_start
            self.nsc_size = nonsecurecall_size
            self.total_s_size = secure_size + nonsecurecall_size
        
        self.__check_region_alignment_to_mpc()


###############################################################################
#
# Instantiate the argument parser.
#

program_description = '''Generates the linker scripts and partiton_{device}.h with
project-defined memory settings. 

NOTE: This script uses binary prefix notation for units when not used by build system
For example: Kilobytes (KB) will be treated as Kibibytes (KiB).

Note: All these arguments will be set with default values according to the {device}.mk file.
If any of these values deviate from the default values (typically '0'), then this
script will assume it was changed by the project-owner.
'''

parser = argparse.ArgumentParser(description=program_description)

# Add required arguments.
parser.add_argument('maxim_path', type=str, help=f'Path to MSDK.')
parser.add_argument('secure_proj_path', type=str, help=f'Path to secure project to store generated secure linker script and partition_{TARGET.lower()}.h.')
parser.add_argument('nonsecure_proj_path', type=str, help='Path to non-secure project to store generated non-secure linker script.')
parser.add_argument('secure_flash_size',
                    type=str,
                    help='Size of Secure Flash as a hexadecimal.')
parser.add_argument('nonsecure_flash_size',
                    type=str,
                    help='Size of Non-Secure Flash as a hexadecimal.')
parser.add_argument('secure_sram_size',
                    type=str,
                    help='Size of Secure SRAM as a hexadecimal.')
parser.add_argument('nonsecure_sram_size',
                    type=str,
                    help='Size of Non-Secure SRAM as a hexadecimal.')

# Add option positional arguments if user wants more fine-grain control.
parser.add_argument('nonsecurecall_region_size',
                    type=str,
                    help='Size of Non-Secure Callable Region as a hexadecimal.')
parser.add_argument('secure_flash_start',
                    type=str,
                    help='Starting address of Secure Flash as a hexadecimal.')
parser.add_argument('nonsecure_flash_start',
                    type=str,
                    help='Starting address of Non-Secure Flashas a hexadecimal.')
parser.add_argument('secure_sram_start',
                    type=str,
                    help='Starting address of Secure SRAM as a hexadecimal.')
parser.add_argument('nonsecure_sram_start',
                    type=str,
                    help='Starting address of Non-Secure SRAM as a hexadecimal.')
parser.add_argument('execute_code_mem',
                    type=str,
                    help='Execute code from Flash or SRAM. Two possible options: "FLASH" or "SRAM"',
                    default="FLASH")

# Parse incoming arguments.
args = parser.parse_args()

###############################################################################
#
# Format console output.
#
print(f"> ****************************************************************************")
print(f"> * Project-owner is using custom memory settings set in project.mk:")
print(f"> * USE_CUSTOM_MEMORY_SETTINGS=1")
print(f"> *")
print(f"> * Preparing (generate/update) Linker Scripts and partition_{TARGET.lower()}.h")
print(f"> * file with user-configured memory settings in project.mk")
print(f"> ****************************************************************************")


###############################################################################
#
# Check arguments and set up memory settings required for generated files.
#
# Prepare sizes for easier calculation checks.
#

# Required arguments
SECURE_FLASH_SIZE = string_to_integer_bytes(args.secure_flash_size)
NONSECURE_FLASH_SIZE = string_to_integer_bytes(args.nonsecure_flash_size)
SECURE_SRAM_SIZE = string_to_integer_bytes(args.secure_sram_size)
NONSECURE_SRAM_SIZE = string_to_integer_bytes(args.nonsecure_sram_size)

# If one memory start argument was set (not '0'), then all of them must be set.
start_check = [string_to_integer_bytes(args.secure_flash_start), string_to_integer_bytes(args.nonsecure_flash_start), string_to_integer_bytes(args.secure_sram_start), string_to_integer_bytes(args.nonsecure_sram_start)]
check_flag = 0
for start in start_check:
    if start != 0:
        check_flag += 1

if check_flag != 0 and check_flag != len(start_check):
    print(f"> [ERROR]: All the starting addresses of Secure/Non-Secure regions must be set by developer in project.mk: S_FLASH_START, NS_FLASH_START, S_SRAM_START, NS_SRAM_START")
    sys.exit(1)

# Save starting adddresses of each region.
if check_flag != 0:
    SECURE_FLASH_START = string_to_integer_bytes(args.secure_flash_start)
    NONSECURE_FLASH_START = string_to_integer_bytes(args.nonsecure_flash_start)
    SECURE_SRAM_START = string_to_integer_bytes(args.secure_sram_start)
    NONSECURE_SRAM_START = string_to_integer_bytes(args.nonsecure_sram_start)
else:
    SECURE_FLASH_START = 0x11000000 # Will occupy Flash first with lower starting address.
    NONSECURE_FLASH_START = (SECURE_FLASH_START + SECURE_FLASH_SIZE) & ~(1<<28) # Occupies second half of Flash.
    SECURE_SRAM_START = 0x30000000 # Occupies SRAM0-2.
    NONSECURE_SRAM_START = (SECURE_SRAM_START + SECURE_SRAM_SIZE) & ~(1<<28) # Occupies SRAM3-4.

# Check memory constraints.
if (SECURE_FLASH_SIZE + NONSECURE_FLASH_SIZE) != PHY_FLASH_SIZE:
    print(f"> [ERROR]: Total Secure and Non-Secure Flash Region sizes exceed entire physical Flash size: {integer_bytes_to_string_KiB(PHY_FLASH_SIZE)}")
    sys.exit(1)

if (SECURE_SRAM_SIZE + NONSECURE_SRAM_SIZE) != PHY_SRAM_SIZE:
    print(f"> [ERROR]: Total Secure and Non-Secure SRAM Region sizes exceed entire physical SRAM size: {integer_bytes_to_string_KiB(PHY_SRAM_SIZE)}")
    sys.exit(1)

# Might be future support to select if NSC would be in same memory type as main code,
#   but for now, they are lumped together.
EXECUTE_CODE_FROM_FLASH = False
IS_NSC_REGION_IN_FLASH = False
EXECUTE_CODE_FROM_SRAM = False
IS_NSC_REGION_IN_SRAM = False

if args.execute_code_mem == "FLASH":
    EXECUTE_CODE_FROM_FLASH = True
    IS_NSC_REGION_IN_FLASH = True
    print(f"> - Main code will execute from Flash.")

elif args.execute_code_mem == "SRAM":
    EXECUTE_CODE_FROM_SRAM = True
    IS_NSC_REGION_IN_SRAM = True
    print(f"> - Main code will execute from SRAM.")

else:
    print(f"> [ERROR]: EXECUTE_CODE_MEM not properly set. Possible options are: FLASH or SRAM")
    sys.exit(1)

# NSC Size. Check to see if project owner set the NSC size.
if string_to_integer_bytes(args.nonsecurecall_region_size) != 0:
    NONSECURECALL_REGION_SIZE = string_to_integer_bytes(args.nonsecurecall_region_size)
else: # if not set by user, then set NSC region size to 8KiB.
    NONSECURECALL_REGION_SIZE = string_to_integer_bytes("8KiB")
    print(f"> - Size for Non-Secure Callable (NSC) Region was not set in project.mk: NSC_SIZE")
    print(f"> - Defaulting size to 8KiB.")

# NSC Region Information that users should be aware of.
NONSECURECALL_FLASH_START = 0
NONSECURECALL_SRAM_START = 0

if args.execute_code_mem == "FLASH":
    print(f"> - Placing NSC Region at the end of Secure Flash Region.")
    print(f"> - Usable Secure Flash Region size is now {integer_bytes_to_string_KiB(NONSECURECALL_REGION_SIZE)} smaller.")
    NONSECURECALL_FLASH_START = SECURE_FLASH_START + SECURE_FLASH_SIZE - NONSECURECALL_REGION_SIZE
    SECURE_FLASH_SIZE = SECURE_FLASH_SIZE - NONSECURECALL_REGION_SIZE

elif args.execute_code_mem == "SRAM":
    print(f"> - Placing NSC Region at the end of Secure SRAM Region.")
    print(f"> - Usable Secure SRAM Region size is now {integer_bytes_to_string_KiB(NONSECURECALL_REGION_SIZE)} smaller.")
    NONSECURECALL_SRAM_START = SECURE_SRAM_START + SECURE_SRAM_SIZE - NONSECURECALL_REGION_SIZE
    SECURE_SRAM_SIZE = SECURE_SRAM_SIZE - NONSECURECALL_REGION_SIZE

###############################################################################
#
# Verify Flash and SRAM settings.
#

FLASH = MemorySettings("FLASH", PHY_FLASH_INST_NUM, PHY_FLASH_BASE_INST, PHY_FLASH_SIZE_INST, MPC_FLASH_BLOCK_SIZE, EXECUTE_CODE_FROM_FLASH)

SRAM = MemorySettings("SRAM", PHY_SRAM_INST_NUM, PHY_SRAM_BASE_INST, PHY_SRAM_SIZE_INST, MPC_SRAM_BLOCK_SIZE, EXECUTE_CODE_FROM_SRAM)

NONSECURECALL_FLASH_SIZE = 0
NONSECURECALL_SRAM_SIZE = 0

# If future support is added, set which type of memory NSC region is located.
#   IS_NSC_REGION_IN_{Memory_Type} configured above.
# This section helps with setting up the memory objects.
if IS_NSC_REGION_IN_FLASH:
    NONSECURECALL_FLASH_SIZE = NONSECURECALL_REGION_SIZE
    NONSECURECALL_REGION_START = NONSECURECALL_FLASH_START
elif IS_NSC_REGION_IN_SRAM:
    NONSECURECALL_SRAM_SIZE = NONSECURECALL_REGION_SIZE
    NONSECURECALL_REGION_START = NONSECURECALL_SRAM_START
else:
    # Should never reach here due to default settings.
    print(f"> [ERROR]: Non-Secure Callable Region does not exist.")
    sys.exit(1)

# More checks happen in this method to make sure everything is aligned.
FLASH.setRegions(SECURE_FLASH_START, SECURE_FLASH_SIZE, NONSECURE_FLASH_START, NONSECURE_FLASH_SIZE, NONSECURECALL_FLASH_START, NONSECURECALL_FLASH_SIZE, IS_NSC_REGION_IN_FLASH)
SRAM.setRegions(SECURE_SRAM_START, SECURE_SRAM_SIZE, NONSECURE_SRAM_START, NONSECURE_SRAM_SIZE, NONSECURECALL_SRAM_START, NONSECURECALL_SRAM_SIZE, IS_NSC_REGION_IN_SRAM)

###############################################################################
#
# Copy the template linker scripts/partion_{device}.h to their appropriate
#   destinations and update them with updated memory settings.
#

def copy_template(source, destination):
    try:
        shutil.copy(source, destination)
    except FileNotFoundError:
        print(f"> [ERROR]: File '{source}' not found.")
    except PermissionError:
        print(f"> [ERROR]: Permission denied. Unable to copy template file over.")
    except Exception as e:
        print(f"> [ERROR]: Exception: {e}")

def replace_string_in_file(path_to_file, dictionary_strings):
    with open(path_to_file, 'r') as file:
        contents = file.read()
    
    for template_string, mem_setting in dictionary_strings.items():
        regex = re.compile(re.escape(template_string))
        contents = re.sub(regex, mem_setting, contents)
    
    with open(path_to_file, 'w') as file:
        file.write(contents)

try:
    SECURE_PARTITION_PATH = os.path.abspath(f"{args.secure_proj_path}/partition_{TARGET.lower()}.h")
    SECURE_LINKER_PATH = os.path.abspath(f"{args.secure_proj_path}/{TARGET.lower()}_s.ld")
    NONSECURE_LINKER_PATH = os.path.abspath(f"{args.nonsecure_proj_path}/{TARGET.lower()}_ns.ld")

    # Check which files exists already. If none exists and there is an error, then
    #   remove them.
    SECURE_PARTITION_ALREADY_EXISTED = False
    if os.path.exists(SECURE_PARTITION_PATH):
        SECURE_PARTITION_ALREADY_EXISTED = True
    
    SECURE_LINKER_ALREADY_EXISTED = False
    if os.path.exists(SECURE_LINKER_PATH):
        SECURE_LINKER_ALREADY_EXISTED = True

    NONSECURE_LINKER_ALREADY_EXISTED = False
    if os.path.exists(NONSECURE_LINKER_PATH):
        NONSECURE_LINKER_ALREADY_EXISTED = True

    SECURE_PARTITION_TEMPLATE_PATH =  os.path.abspath(f"{args.maxim_path}/{SECURE_PARTITION_TEMPLATE_PATH}")
    SECURE_LINKER_TEMPLATE_PATH = os.path.abspath(f"{args.maxim_path}/{SECURE_LINKER_TEMPLATE_PATH}")
    NONSECURE_LINKER_TEMPLATE_PATH = os.path.abspath(f"{args.maxim_path}/{NONSECURE_LINKER_TEMPLATE_PATH}")

    copy_template(SECURE_PARTITION_TEMPLATE_PATH, SECURE_PARTITION_PATH)
    copy_template(SECURE_LINKER_TEMPLATE_PATH, SECURE_LINKER_PATH)
    copy_template(NONSECURE_LINKER_TEMPLATE_PATH, NONSECURE_LINKER_PATH)

    # Update memory settings.
    regex_replace = {
        # Secure Flash
        r'$FLASH_ORIGIN_S$' : f"0x{FLASH.s_start:08x}",
        r'$FLASH_END_S$'    : f"0x{(FLASH.s_start + FLASH.s_size - 1):08x}",
        r'$FLASH_SIZE_S$'   : f"0x{FLASH.s_size:08x}",

        # Non-Secure Flash
        r'$FLASH_ORIGIN_NS$': f"0x{FLASH.ns_start:08x}",
        r'$FLASH_END_NS$'   : f"0x{(FLASH.ns_start + FLASH.ns_size - 1):08x}",
        r'$FLASH_SIZE_NS$'  : f"0x{FLASH.ns_size:08x}",

        # Secure SRAM
        r'$SRAM_ORIGIN_S$'  : f"0x{SRAM.s_start:08x}",
        r'$SRAM_END_S$'     : f"0x{SRAM.s_start + SRAM.s_size - 1:08x}",
        r'$SRAM_SIZE_S$'    : f"0x{SRAM.s_size:08x}",

        # Non-Secure SRAM
        r'$SRAM_ORIGIN_NS$' : f"0x{SRAM.ns_start:08x}",
        r'$SRAM_END_NS$'    : f"0x{(SRAM.ns_start + SRAM.ns_size - 1):08x}",
        r'$SRAM_SIZE_NS$'   : f"0x{SRAM.ns_size:08x}"
    }

    # Mainly for linker script
    if FLASH.is_nonsecurecall_present:
        regex_replace[r'$MEM_ORIGIN_NSC$'] = f"0x{FLASH.nsc_start:08x}"
        regex_replace[r'$MEM_SIZE_NSC$'] = f"0x{FLASH.nsc_start:08x}"
        regex_replace[r'$perm$'] = f" (rx)"
        regex_replace[r'$FLASH_END_S$'] = f"0x{(FLASH.s_start + FLASH.s_size + FLASH.nsc_size - 1):08x}"
    elif SRAM.is_nonsecurecall_present:
        regex_replace[r'$MEM_ORIGIN_NSC$'] = f"0x{SRAM.nsc_start:08x}"
        regex_replace[r'$MEM_SIZE_NSC$'] = f"0x{SRAM.nsc_start:08x}"
        regex_replace[r'$perm$'] = f"(rwx)"
        regex_replace[r'$SRAM_END_S$'] = f"0x{SRAM.s_start + SRAM.s_size + SRAM.nsc_size - 1:08x}"
    else:
        raise ValueError("Unexpected problem with updating NSC regions in linker scripts.")

    # Mainly for linker script
    if FLASH.execute_code_here:
        regex_replace[r'$CODE_MEM_TYPE$'] = f"FLASH"
        # NSC region at the end of FLASH for this string in the linkers 
        regex_replace[r'$MPC_BLOCK_SIZE$'] = f"0x{FLASH.mpc_inst_blk_sizes[-1]:x}"
    elif SRAM.execute_code_here:
        regex_replace[r'$MPC_BLOCK_SIZE$'] = f"FLASH"
        # NSC region at the end of FLASH for this string in the linkers 
        regex_replace[r'$MPC_BLOCK_SIZE$'] = f"0x{SRAM.mpc_inst_blk_sizes[-1]:x}"
    else:
        raise ValueError("Unexpected problem with updating sections in linker scripts.")

    # Update templates
    # Update partition file.
    replace_string_in_file(SECURE_PARTITION_PATH, regex_replace)

    # Update secure linker script.
    replace_string_in_file(SECURE_LINKER_PATH, regex_replace)

    # Update non-secure linker script.
    replace_string_in_file(NONSECURE_LINKER_PATH, regex_replace)

except ValueError as ve:
    print(f"> [ERROR]: {ve}")
except Exception:
    print(f"> [ERROR]: Failed to update templates.")

    # Remove files if they did not exist before.
    if SECURE_PARTITION_ALREADY_EXISTED == False:
        os.remove(SECURE_PARTITION_PATH)
    if SECURE_LINKER_ALREADY_EXISTED == False:
        os.remove(SECURE_LINKER_PATH)
    if NONSECURE_LINKER_ALREADY_EXISTED == False:
        os.remove(NONSECURE_LINKER_PATH)
    
    sys.exit(1)

###############################################################################
#
# Print memory settings for user.
#

if FLASH.is_nonsecurecall_present:
    PRINT_NONSECURECALL_REGION_START = FLASH.nsc_start
    PRINT_NONSECURECALL_REGION_SIZE = FLASH.nsc_size
elif SRAM.is_nonsecurecall_present:
    PRINT_NONSECURECALL_REGION_START = SRAM.nsc_start
    PRINT_NONSECURECALL_REGION_SIZE = SRAM.nsc_size

print(f"> ----------------------------------------------------------------------------")
print(f">  Memory Settings")
print(f"> ----------------------------------------------------------------------------")
print(f"> Secure Flash Start (S_FLASH_START)         : 0x{FLASH.s_start:08x}")
print(f"> Secure Flash Size (S_FLASH_SIZE)           : 0x{FLASH.s_size:08x}")
print(f"> Non-Secure Flash Start (NS_FLASH_START)    : 0x{FLASH.ns_start:08x}")
print(f"> Non-Secure Flash Size (NS_FLASH_SIZE)      : 0x{FLASH.ns_size:08x}")
print(f"> ")
print(f"> Secure SRAM Start (S_SRAM_START)           : 0x{SRAM.s_start:08x}")
print(f"> Secure SRAM Size (S_SRAM_SIZE)             : 0x{SRAM.s_size:08x}")
print(f"> Non-Secure SRAM Start (NS_START)           : 0x{SRAM.ns_start:08x}")
print(f"> Non-Secure SRAM Size (NS_SRAM_SIZE)        : 0x{SRAM.ns_size:08x}")
print(f"> ")
print(f"> Non-Secure Callable Region Start           : 0x{PRINT_NONSECURECALL_REGION_START:08x}")
print(f"> Non-Secure Callable Region Size (NSC_SIZE) : 0x{PRINT_NONSECURECALL_REGION_SIZE:08x}")
print(f"> ")
