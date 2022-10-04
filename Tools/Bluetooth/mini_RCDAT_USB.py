#! /usr/bin/env python3

################################################################################
 # Copyright (C) 2020 Maxim Integrated Products, Inc., All Rights Reserved.
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

## mini_RCDAT_USB.py
 #
 # Sets the attenuation of a Mini-Circuits RCDAT USB attenuator
 #

import usb.core
import usb.util
import argparse
from argparse import RawTextHelpFormatter
import sys

# Set the VID and PID that we're going to try and connect to
vid=0x20ce
pid=0x0023

# Define the min and max attenuation values
minDbm=0
maxDbm=90

# Namespace class used to create function arguments similar to argparse
class Namespace:
    def __init__(self, **kwargs):
        self.__dict__.update(kwargs)

class mini_RCDAT_USB:

    def __init__(self, args):

      # Make sure attenuation is within the defined limits
      try:
         if(float(args.atten) > maxDbm) :
            print("Attenuation parameter is too high, max attenuation: "+str(maxDbm))
            print(descText)
            sys.exit(1)
         if(float(args.atten) < minDbm) :
            print("Attenuation parameter is too low, min attenuation: "+str(minDbm))
            print(descText)
            sys.exit(1)
      except ValueError as err:
         print("Error with attenuation value")
         print(descText)
         sys.exit(1)

      # Find our device
      dev = usb.core.find(idVendor=vid, idProduct=pid)

      if dev is None:
         raise ValueError('Device not found')

      for configuration in dev:
         for interface in configuration:
            ifnum = interface.bInterfaceNumber
            if not dev.is_kernel_driver_active(ifnum):
               continue
            try:
               dev.detach_kernel_driver(ifnum)
            except usb.core.USBError as e:
               pass

      # set the active configuration. with no args we use first config.
      dev.set_configuration()

      dev.reset()

      # Get the SN
      SerialN=""
      dev.write(1,"*:SN?") 
      sn=dev.read(0x81,64) 
      i=1
      while (sn[i]<255 and sn[i]>0):  
         SerialN=SerialN+chr(sn[i])
         i=i+1

      # Get the model number
      ModelN=""
      dev.write(1,"*:MN?") 
      mn=dev.read(0x81,64) 
      i=1
      while (mn[i]<255 and mn[i]>0):  
         ModelN=ModelN+chr(mn[i])
         i=i+1 

      # Get the firmware version
      Fw=""
      dev.write(1,"*:FIRMWARE?") 
      sn=dev.read(0x81,64) 
      i=1
      while (sn[i]<255 and sn[i]>0):  
         Fw=Fw+chr(sn[i])
         i=i+1

      retval=0

      # Set the attenuation
      attenCmdString="*:SETATT="+str(args.atten)+";"
      print(attenCmdString)
      dev.write(1,attenCmdString) 
      resp=dev.read(0x81,64)
      i=1
      AttResp=""
      while (resp[i]<255 and resp[i]>0):  
         AttResp=AttResp+chr(resp[i])
         i=i+1 
      if (AttResp != "1"):
         print("Error setting attenuation")
         retval=1

      # Check the attenuation
      getAttenString="*:ATT?"
      print(getAttenString)
      dev.write(1,getAttenString) # return attenuation value
      resp=dev.read(0x81,64)
      i=1
      AttResp=""
      while (resp[i]<255 and resp[i]>0):  
         AttResp=AttResp+chr(resp[i])
         i=i+1 
      print("Attenuation: " + AttResp)

      usb.util.release_interface(dev, 0)
      # dev.close()

if __name__ == '__main__':

   # Setup the command line description text
   descText = """
   Mini-Circuits RCDAT USB configuration tool.

   This tool is used to set the attenuation of a Mini Circuits RCDAT, using the USB interface.
   Attenuation value must be a decimal number between """+str(minDbm)+""" and """+str(maxDbm)+"""

   VID=0x"""+'%04X'%vid+"""
   PID=0x"""+'%04X'%pid

   # Parse the command line arguments
   parser = argparse.ArgumentParser(description=descText, formatter_class=RawTextHelpFormatter)
   parser.add_argument('atten',help='attenuation in dBm, 0.25 dB resolution')

   args = parser.parse_args()

   print("Mini-Circuits RCDAT USB configuration tool")
   print("Attenuation: "+str(args.atten))

   # Set the attenuation
   atten = mini_RCDAT_USB(args)

   sys.exit(0)
