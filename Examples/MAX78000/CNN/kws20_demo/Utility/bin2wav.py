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
import os
import argparse
import soundfile as sf
import sounddevice as sd
import numpy as np
import matplotlib.pyplot as plt

def readbin(input, output):

    data = np.fromfile(input, dtype = 'int8')
    sd.play(data, 16384)
    print (data)
    plt.plot(data)
    plt.show()

    # save as .np
    np.save(output[:-4], data)

    sf.write(output, 100 * data.astype('int16'), len(data))

def command_parser():
    """
    Return the argument parser
    """
    parser = argparse.ArgumentParser(description='Convert bin to audio wav')
    parser.add_argument('-i', '--input', type=str, required = True,
                        help='input bin file')

    parser.add_argument('-o', '--output', type=str, required = False, default = 'out.wav',
                        help='output wav')
    return parser.parse_args()

if __name__ == "__main__":
    command = command_parser()
    readbin(command.input, command.output)