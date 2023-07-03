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
from datetime import datetime
import fcntl
import logging
import os
from pprint import pprint
import subprocess
import usb.core
import usb.util
import sys

logging.basicConfig(filename=os.path.expanduser('~/Workspace/Resource_Share/Logs/mc_rf_sw.log'), 
                    level=logging.DEBUG,
                    format='%(message)s')


WITH_PRINT = False

# Equivalent of the _IO('U', 20) constant in the linux kernel.
USBDEVFS_RESET = ord('U') << (4*2) | 20


def PRINT(msg):
    if WITH_PRINT:
        print(msg)
        logging.debug(f'{datetime.now()} {msg}')


class Namespace:
    def __init__(self, **kwargs):
        self.__dict__.update(kwargs)


class McRfSw:
    """
    class for Mini-Circuits Solid State RF Switches.
    model: USB-1SP16T-83H
    model: USB-1SP8T-63H
    """
    available_models = ("USB-1SP16T-83H", "USB-1SP8T-63H")
    sw_type = ("SP16T", "SP8T")
    
    def __init__(self, args):
        global WITH_PRINT
        
        if args.debug:
            WITH_PRINT = True
        else:
            WITH_PRINT = False
            
        PRINT(f'{args}')

        if args.model not in McRfSw.available_models:
            msg = f"Invalid model: {args.model}"
            raise Exception(msg)
        self.model = args.model
        
        # find the right device by the model name
        self.dev = self.find_the_device(self.model)
        if self.dev is None:
            msg = f"Fail to find the device for {args.model}"
            raise Exception(msg)
        
        # get device sn
        self.sn = ""
        cmd = "*:SN?"
        PRINT(f'cmd: {cmd}')
        self.dev.write(1, cmd)
        sn = self.dev.read(0x81, 64)
        PRINT(f'Received: {sn}')
        i = 1
        while 255 > sn[i] > 0 and i <= len(sn):
            self.sn = self.sn + chr(sn[i])
            i = i + 1
        PRINT(f'Device SN: {self.sn}\n')  # 12210300026
        
        if self.model == McRfSw.available_models[0]:
            self.model_index = 0
        else:
            self.model_index = 1
        
        # get firmware
        self.firmware = ""
        cmd = "*:FIRMWARE?"
        PRINT(f'cmd: {cmd}')
        self.dev.write(1, cmd)
        fw = self.dev.read(0x81, 64)
        PRINT(f'Received: {fw}')
        i = 1
        while 255 > fw[i] > 0 and i <= len(fw):
            self.firmware = self.firmware + chr(fw[i])
            i = i + 1
        PRINT(f'Firmware: {self.firmware}\n')

        if args.op is None:
            exit(0)
        
        self.dev.reset()
        
        if args.op.lower() == "get":
            state = self.get_sw_state()
            msg = f'Current state: {state}'
            logging.debug(msg)
            print(msg)
        elif args.op.lower() == "set":
            self.set_sw_state(args.state)
        elif args.op.lower() == "reset":
            PRINT("")
            PRINT("Reset the device.")
            #self.reset_device()
        else:
            raise ValueError("Invalid op.")

        usb.util.release_interface(self.dev, 0)
        
    def find_the_device(self, model):
        """find the right device by its model name
        """
        all_devs = usb.core.find(idVendor=0x20ce, idProduct=0x0022, find_all=True)
        for d in all_devs:
            # check configuration in this device
            self.dettach_and_config(d)
            
            # get model name
            model_name = ""
            cmd = "*:MN?"
            PRINT(f'cmd: {cmd}')
            d.write(1, cmd)
            mn = d.read(0x81, 64)
            PRINT(f"Received: {mn}")
            i = 1
            while 255 > mn[i] > 0 and i <= len(mn):
                model_name = model_name + chr(mn[i])
                i = i + 1
            PRINT(f'Model name: {model_name}\n')
            
            if model_name == model:
                return d
            else:
                PRINT('--- NOT THIS DEVICE ---')
        return None
   
    def dettach_and_config(self, d):
        for configuration in d:
            PRINT("")
            PRINT("configuration:")
            PRINT(configuration)
            
            for interface in configuration:
                PRINT("")
                PRINT("interface:")
                PRINT(interface)
                if_num = interface.bInterfaceNumber
                if not d.is_kernel_driver_active(if_num):
                    continue
                
                try:
                    d.detach_kernel_driver(if_num)
                except usb.core.USBError as e:
                    print(f'Interface Number: {if_num}')
                    print(e)
    
        # set the active configuration. with no args we use first config.
        d.set_configuration()
       
    def set_sw_state(self, state):
        """Switch State Commands / Queries
            :[Sw_Type]:[Sw_Channel]:STATE:[Sw_State]
            
            :arg
                state
            :return
                status  0:command failed, 1: command completed successfully
        """
        curr_state = self.get_sw_state()
        PRINT("")
        msg = f'Current state: {curr_state}'
        logging.debug(msg)
        print(msg)
        
        if curr_state == str(state):
            return curr_state
        
        cmd = f'*:{McRfSw.sw_type[self.model_index]}:STATE:{state}'
        PRINT(f'cmd: {cmd}')
        self.dev.write(1, cmd)
        resp = self.dev.read(0x81, 64)
        PRINT(f'Received: {resp}')
        i = 1
        set_resp = ""
        while 255 > resp[i] > 0 and i <= len(resp):
            set_resp = set_resp + chr(resp[i])
            i = i + 1
        if set_resp == "1":
            PRINT("Success")
            
            new_state = self.get_sw_state()
            PRINT("")
            msg = f'    New state: {new_state}'
        else:
            msg = 'FAILED!'
        
        logging.debug(msg)
        print(msg)
            
        return set_resp
    
    def get_sw_state(self):
        """get switch state
        
        """
        cmd = f'*:{McRfSw.sw_type[self.model_index]}:STATE?'
        PRINT(f'cmd: {cmd}')
        self.dev.write(1, cmd)
        state_ret = self.dev.read(0x81, 64)
        PRINT(f'Received: {state_ret}')
        i = 1
        resp = ""
        while 255 > state_ret[i] > 0 and i <= len(state_ret):
            resp = resp + chr(state_ret[i])
            i = i + 1
        PRINT(f'Resp: {resp}')
        return resp

    def reset_device(self):
        """reset the RF switch
        """
        proc = subprocess.Popen(['lsusb'], stdout=subprocess.PIPE)
        out = proc.communicate()[0]
        out = out.decode('utf-8')
        lines = out.split('\n')
        for line in lines:
            if "Minicircuits I/O Controller" in line:
                parts = line.split()
                bus = parts[1]
                dev = parts[3][:3]
                dev_path = '/dev/bus/usb/%s/%s' % (bus, dev)
                PRINT(f'dev_path: {dev_path}')

                fd = os.open(dev_path, os.O_WRONLY)
                try:
                    fcntl.ioctl(fd, USBDEVFS_RESET, 0)
                finally:
                    os.close(fd)


if __name__ == "__main__":
    WITH_PRINT = 1

    # Setup the command line description text
    descText = """
       Mini-Circuits RF Switch control.
    """
    
    # Parse the command line arguments
    parser = argparse.ArgumentParser(description=descText, formatter_class=RawTextHelpFormatter)
    parser.add_argument('--model', help='Model: USB-1SP16T-83H, or USB-1SP8T-63H')
    parser.add_argument('--op', help='get, set, reset')
    parser.add_argument('--state', help='state')
    parser.add_argument('--debug', action="store_true", help='display debug info')

    args = parser.parse_args()
    
    if args.debug:
        PRINT("\n\n\n")
        print("Mini-Circuits RF Switch Control Tool")
        print(f'Input arguments: {args}')
    
    McRfSw(args)
    
    
