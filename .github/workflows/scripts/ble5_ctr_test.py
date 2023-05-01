#! /usr/bin/env python3

################################################################################
# Copyright (C) 2023 Maxim Integrated Products, Inc., All Rights Reserved.
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

# ble5_ctr_test.py
#
# Description: 
#  CI test for the MSDK repo Examples/MAX.../BLE5_ctr projects.
#

import argparse
from io import StringIO
import json
import os
from pprint import pprint
import socket
import sys
from time import sleep


def get_inputs():
    parser = argparse.ArgumentParser()
    parser.add_argument("--msdk", help="msdk repo")
    parser.add_argument("--chip", help="chip type in uppercase, like MAX32655, MAX32665, MAX32690")
    parser.add_argument("--type", help="board type, like EvKit_V1, WLP_V1")
    parser.add_argument("--time", help="test time, used for log file name etc.")

    args = parser.parse_args()
    print(f'\nuser inputs:')
    pprint(f'{vars(args)}')

    return args


def test_on_board(user_args):
    """
        results:
            0: success
            1: fail
    """
    #
    # get hci and console ports from the configuration files
    #
    chip_and_board = f'{user_args.chip.lower()}_{user_args.type.lower()}'
    print(f'chip and board type: {chip_and_board}')
    file = os.path.expanduser("~/Workspace/ci_config/ble5_ctr_test.json")
    obj = json.load(open(file))
    hostname = socket.gethostname()
    brd2 = obj["ble5_ctr_test.yml"][hostname][chip_and_board]["board2"]
    print(f'board 2: {brd2}')

    file = os.path.expanduser("~/Workspace/ci_config/boards_config.json")
    obj = json.load(open(file))

    hci2 = obj[brd2]["hci_id"]
    print(f'board 2 HCI: {hci2}')

    con2 = obj[brd2]["con_id"]
    if con2.lower() == "x":
        con2 = ""
    print(f'board 2 CON: {con2}')

    msdk = os.path.expanduser(user_args.msdk)
    bt_tools_path = os.path.join(msdk, "Tools/Bluetooth")
    sys.path.append(bt_tools_path)
    from BLE_hci import BLE_hci, Namespace
    
    slv_hci = BLE_hci(Namespace(serialPort=hci2, baud=115200, monPort=con2))
    
    # capture the output to the console
    original_stdout = sys.stdout
    console_output = StringIO()
    sys.stdout = console_output
    
    # send reset command to the device
    slv_hci.resetFunc(None)
    
    # retrieve the output
    console_output_str = console_output.getvalue()
    
    # restore the stdout
    sys.stdout = original_stdout
    
    print(f'console_output_str:\n{console_output_str}')
    
    print("#-------------------------------------------------------------------")
    if console_output_str.find("< 040E0401030C00") == -1:
        print("# FAILED TO GET CORRECT RESPONSE FROM THE DEVICE!")
        res = 1
    else:
        print("# GOT THE CORRECT RESPONSE FROM THE DEVICE!")
        res = 0
    print("#-------------------------------------------------------------------")
    
    return res


if __name__ == "__main__":
    inputs = get_inputs()

    res = test_on_board(inputs)

    sys.exit(res)