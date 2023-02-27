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
import os
import argparse
import soundfile as sf
import numpy as np
import serial

def RX_Binary(ser, length = 16384):
    """
    Receives 16K binary samples for serial port
    """
    rx = np.empty(length, dtype=np.int8)
    cnt = 0
    while cnt < length:
        temp = ser.read()
        if temp:
            rx[cnt] = temp[0]
            cnt += 1
    return rx

def Capture_Serial(COMport, fname, length=16384):
    """
    Initialize the COM port, waits for  keyword 'START', then receives 16K binary samples,
    waits for 'END' and stores in a .npy file to be used for evaluation
    """
    ser = serial.Serial(COMport)
    ser.baudrate = 115200
    ser.timeout = 0.001

    print("waiting for serial data")
    while True:
        rx = ser.readline()
        line = rx[0:len(rx) - 1].decode(errors="ignore")
        if line == "":
            continue
        if line.startswith('Detected word') or line.startswith('LOW'):
            print(line)
        elif line.startswith('START'):
            # Receive binary
            data = RX_Binary(ser)
            print("Received serial data")
            continue
        elif line.startswith('END'):
            np.save(fname[:-4] + '.npy', data)
            ser.close()
            fname = fname[:-4] + '_' + str(counter) + '.wav'
            print(f"Saved serial data {data} to {fname}")
            sf.write(os.path.join("", fname), 100*data.astype('int16'), length)
            break
    return data

def command_parser():
    """
    Return the argument parser
    """
    parser = argparse.ArgumentParser(description='Captures serial bin file')
    parser.add_argument('-c', '--comport', type=str, required = False, default = 'COM12',
                        help='serial port')

    parser.add_argument('-o', '--output', type=str, required = False, default = 'out.wav',
                        help='output wav')
    
    return parser.parse_args()

if __name__ == "__main__":
    command = command_parser()
    counter = 0
    while True:
        data = Capture_Serial(COMport = command.comport, fname = command.output)
        counter += 1    

