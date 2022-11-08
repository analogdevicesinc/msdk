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

def convertAll(topDir):
    """
    Walk directories from the root and convert all sample audio file to wave (.wav) files
    - skips if the file has already been converted.
    - skips if the file size is not exactly 16384.
    - returns number of files converted.
    """
    numConverted = 0
    for root, dirs, files in os.walk(topDir):
        for filename in files:
            input = os.path.join(root, filename) 
            if os.path.splitext(input)[-1] == '' and os.path.getsize(input) == 16384:
                output = input + ".wav"
            if not os.path.isfile(output):
                data = np.fromfile(input, dtype = 'int8')
                sf.write(output, 100 * data.astype('int16'), len(data))
                numConverted += 1
    return numConverted

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
    parser.add_argument('-i', '--input', type=str, required = False,
                        help='input bin file')

    parser.add_argument('-o', '--output', type=str, required = False, default = 'out.wav',
                        help='output wav')
    
    parser.add_argument('-a', '--all', required = False, action="store_true",
                        help='walk all directories and convert all files')
    
    parser.add_argument('-d', '--directory', type = str, required = False, default = ".",
                        help='top directory')
    return parser.parse_args()

if __name__ == "__main__":
    command = command_parser()
    if command.all is False:
        readbin(command.input, command.output)
    else:
        print(f"{convertAll(command.directory)} files are converted!")