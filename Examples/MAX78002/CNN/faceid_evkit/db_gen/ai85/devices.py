###############################################################################
 #
 # Copyright (C) 2022-2023 Maxim Integrated Products, Inc. All Rights Reserved.
 # (now owned by Analog Devices, Inc.),
 # Copyright (C) 2023 Analog Devices, Inc. All Rights Reserved. This software
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
 ##############################################################################
"""
Part number and device type conversion
"""
import argparse


def device(astring):
    """
    Take die type, or part number, and return the die type.
    """
    s = astring.lower()

    if s.startswith('max'):
        s = s[3:]  # Strip 'MAX' from part number
    elif s.startswith('ai'):
        s = s[2:]  # Strip 'AI' from die type

    try:
        num = int(s)
    except ValueError as exc:
        raise argparse.ArgumentTypeError(astring, 'is not a supported device type') from exc
    if num in [84, 85, 87]:  # Die types
        dev = num
    elif num == 78000:  # Part numbers
        dev = 85
    elif num == 78002:
        dev = 87
    else:
        raise argparse.ArgumentTypeError(astring, 'is not a supported device type')

    return dev


def partnum(num):
    """
    Return part number for a die type.
    """
    if num == 84:
        return 'AI84'
    if num == 85:
        return 'MAX78000'
    if num == 87:
        return 'MAX78002'

    raise RuntimeError(f'Unknown die type {num}')
