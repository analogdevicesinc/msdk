#! /usr/bin/env python3

################################################################################
# Copyright (C) 2023 Analog Devices, Inc., All Rights Reserved.
#
# Permission is hereby granted, free of charge, to any person obtaining a
# copy of this software and associated documentation files (the "Software"),
# to deal in the Software without restriction, including without limitation
# the rights to use, copy, modify, merge, publish, distribute, sublicense,
# and/or sell copies of the Software, and to permit persons to whom the
# Software is furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included
# in all copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS
# OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
# MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.
# IN NO EVENT SHALL MAXIM INTEGRATED BE LIABLE FOR ANY CLAIM, DAMAGES
# OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE,
# ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR
# OTHER DEALINGS IN THE SOFTWARE.
#
# Except as contained in this notice, the name of Maxim Integrated
# Products, Inc. shall not be used except as stated in the Maxim Integrated
# Products, Inc. Branding Policy.
#
# The mere transfer of this software does not imply any licenses
# of trade secrets, proprietary technology, copyrights, patents,
# trademarks, maskwork rights, or any other form of intellectual
# property whatsoever. Maxim Integrated Products, Inc. retains all
# ownership rights.
#
###############################################################################

import argparse
from argparse import RawTextHelpFormatter
from curses.ascii import isdigit
import os
import sys

LIMIT = 30.0

WITH_PRINT = False


def PRINT(msg):
    if WITH_PRINT:
        print(msg)


def parse_input():
    """get the input arguments
    """
    global WITH_PRINT

    # Setup the command line description text
    descText = """
       Script used to check the PER test results.
    """
    
    # Parse the command line arguments
    parser = argparse.ArgumentParser(description=descText, formatter_class=RawTextHelpFormatter)
    parser.add_argument('--csv', default='', help='The csv file of the PER test results.')
    parser.add_argument('--debug', action="store_true", help='display debug info')

    args = parser.parse_args()
    if args.debug:
        WITH_PRINT = True
    else:
        WITH_PRINT = False

    PRINT(f'Input arguments: {args}')

    return args


def check_results(res_file):
    """read the result file and check the PER

        return:
            0: PASS
            1: FAIL
    """
    global WITH_PRINT

    if not os.path.exists(res_file):
        return 1
    
    with open(res_file) as file:
        while (line := file.readline().rstrip()):
            PRINT(line)
            temp = line.split(',')
            #PRINT(temp)
            if len(temp) >= 2:
                if temp[-1].replace('.', '', 1).isdigit():
                    per = float(temp[-1])                    
                    if per > LIMIT:
                        PRINT(f'FAILED: {per}')
                        return 2
                if temp[-2].replace('.', '', 1).isdigit():
                    per = float(temp[-2])
                    if per > LIMIT:
                        PRINT(f'FAILED: {per}')
                        return 2

    return 0


input = parse_input()
#input.csv = "/home/ying-cai/temp/simple-2023-02-09_23-08-41_max32655.csv"
#WITH_PRINT = True
res = check_results(input.csv)
if res > 0:
    print("FAILED!")
    sys.exit(1)




