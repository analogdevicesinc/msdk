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
#

'''
    This is an example application executing remote
    cifar-10 inferences on a MAX78000 evaluation kit.

    CIFAR-10 classifications by index:
    0 airplane
    1 automobile
    2 bird
    3 cat
    4 deer
    5 dog
    6 frog
    7 horse
    8 ship
    9 truck
'''

import os
import sys
import argparse
import serial
import ft4222
import ft4222.SPI as SPI
import ft4222.SPIMaster as SPIM
import PIL.Image as Image

# Start - Match these with MAX78000 application
SERIAL_BAUD = 115200
IMG_WIDTH   = 32
IMG_HEIGHT  = 32
# End

SERIAL_TOUT = 5 # Seconds
IMG_MODE    = "RGB"


def print_result(filename, result):
    """ Print formatted result """
    print("{0}\t{1}".format(filename, result), end = '')


def main():
    """ main function """
    parser = argparse.ArgumentParser(description = "Test PNG images with cifar-10 based example.",
                                     formatter_class = argparse.RawTextHelpFormatter)
    parser.add_argument("-p", "--path",
                        required = True,
                        help = "Relative or absolute path to the directory of test images.")
    parser.add_argument("-d", "--device",
                        required = True,
                        help = "Serial device connected to MAX78000.\n"
                               "Linux: /dev/ttyUSB0\n"
                               "Windows: COM1")

    args = parser.parse_args()

    abs_path = os.path.realpath(args.path)
    if not os.path.exists(abs_path):
        print("ERROR Directory does not exists: {0}".format(abs_path))
        sys.exit()

    print("\nTesting images from: {0}\n".format(abs_path))

    # Open device with default description
    spim = ft4222.openByDescription('FT4222 A')

    spim.spiMaster_Init(SPIM.Mode.SINGLE,
                        SPIM.Clock.DIV_8,
                        SPI.Cpol.IDLE_LOW,
                        SPI.Cpha.CLK_LEADING,
                        SPIM.SlaveSelect.SS0)

    # Serial port must be open before end of SPI transaction to avoid missing characters
    sport = serial.Serial(args.device, SERIAL_BAUD, timeout = SERIAL_TOUT)

    for filename in sorted(os.listdir(abs_path)):
        # Ignore all other files
        if not filename.endswith('.png'):
            continue

        with Image.open(os.path.join(abs_path, filename)) as img:
            if not img.mode == IMG_MODE:
                print_result(filename,
                             "Skipped, image mode must be {0}\n".format(IMG_MODE))
                continue

            if not img.size == (IMG_WIDTH, IMG_HEIGHT):
                print_result(filename,
                             "Skipped, image size must be {0}x{1}\n".format(IMG_WIDTH, IMG_HEIGHT))
                continue

            image_data = img.getdata()

        # Convert list of tuples to flat list
        image_data = [color_value for pixel in image_data for color_value in pixel]

        # Send image to MAX78000 SPI slave
        _ = spim.spiMaster_SingleWrite(bytes(image_data), True)

        # Wait for the test result
        # read() blocks until SERIAL_TOUT
        result = []
        while 1:
            char = sport.read(1)
            if char == b'':
                result = "Timeout"
                break
            result.append(char.decode('utf-8'))
            if char == b'\n':
                result = "".join(result)
                break

        print_result(filename, result)

    sport.close()
    spim.close()


if __name__ == "__main__":
    main()
