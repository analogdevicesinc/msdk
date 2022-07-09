################################################################################
 # Copyright (C) 2022 Maxim Integrated Products, Inc., All Rights Reserved.
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
#

'''
    This is an example application sending images to UNet-demo
'''

import os
import sys
import argparse
import ft4222
import ft4222.SPI as SPI
import ft4222.SPIMaster as SPIM
from PIL import Image
import time

# Start - Match these with MAX78000 application
SERIAL_BAUD = 115200
IMG_WIDTH   = 80
IMG_HEIGHT  = 80
# End

SERIAL_TOUT = 1 # Seconds
IMG_MODE    = "RGB"


def print_result(filename, result):
    """ Print formatted result """
    print("{0}\t{1}".format(filename, result), end = '')


def main():
    """ main function """
    parser = argparse.ArgumentParser(description = "Send an image to UNet",
                                     formatter_class = argparse.RawTextHelpFormatter)
    parser.add_argument("-i", "--inp",
                        required = True,
                        help = "Test image")
    args = parser.parse_args()

    print("\nTesting image: {image}\n")

    # Open device with default description
    spim = ft4222.openByDescription('FT4222 A')

    spim.spiMaster_Init(SPIM.Mode.SINGLE,
                        SPIM.Clock.DIV_8,
                        SPI.Cpol.IDLE_LOW,
                        SPI.Cpha.CLK_LEADING,
                        SPIM.SlaveSelect.SS0)

    filename = args.inp
    # Ignore all other files
    img = Image.open(filename)

    if not img.mode == IMG_MODE:
        print_result(filename,
                     "Skipped, image mode must be {0}\n".format(IMG_MODE))
        exit()

    if not img.size == (IMG_WIDTH, IMG_HEIGHT):
        print(f'Image resized to ({IMG_WIDTH},{IMG_HEIGHT})')
        img = img.resize((IMG_WIDTH, IMG_HEIGHT))

    image_data = img.getdata()

    # Convert list of tuples to flat list
    image_data = [color_value for pixel in image_data for color_value in pixel]
    # Delay needed for application on evkit
    time.sleep(0.5)

    image_data32 = [];
    # Make data 32bit RGB
    for cnt in range(0,len(image_data)):
        image_data32.append(image_data[cnt])
        if (cnt + 1) % 3 == 0:
            image_data32.append(0)
    # Send image to MAX78000 SPI slave
    _ = spim.spiMaster_SingleWrite(bytes(image_data32), True)

    print(f'{len(image_data)} bytes sent: {filename}')
   # input("Press Enter to continue...")

    spim.close()


if __name__ == "__main__":
    main()
