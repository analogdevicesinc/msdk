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

"""
This file implements a simple console program for communicating
with the 'ImgCapture' firmware.

Installation:
1. Install Python 3
2. pip install -r requirements.txt

Run "python console.py -h" for help, or see the readme.

"""

from serial import Serial
from threading import Thread, Lock
import traceback
import re
from imgConverter import convert
import argparse
import cv2
from pathlib import Path

# Utility printer for printing a tracking console prompt "$"
# after the message.  Is there a better way to do this?  Probably...
def _print(msg: str):
    print(f"\r{msg}", end="\n\r$ ")

class CameraIFConsole():
    def __init__(self, port, baudrate=921600, timeout=5):
        self.thr_get_input = Thread(target=self.get_input)
        self.kill_get_input = False
        self.thr_serial = Thread(target=self.run_serial_interface)
        self.kill_serial = False

        self.input = ""
        self.lock_input = Lock()

        self.s = Serial(port=port, baudrate=baudrate, timeout=timeout)
        self.s.reset_input_buffer()
        self.s.reset_output_buffer()

        print(f"Started ImgCapture console and opened {port}")
        self.input = "help" # Queue up a help command in case we're connecting to firmware that's already initialized
        self.thr_serial.start()
        self.thr_get_input.start()
        

    """
    This blocking function continuously polls user input and writes it 
    to a thread-safe (locked) member variable 'input'.  

    It can be killed by setting the member "kill_get_input" variable to True.
    It will also die (gracefully) if it throws any internal exceptions.
    """
    def get_input(self):
        try:
            while not self.kill_get_input:
                _input = input()

                if _input == "quit" or _input == "q":
                    self.quit()
                # elif _input == "help" or _input == "h":
                #     self.help()
                #     _input = ""
                elif "set-reg" in _input:
                    expr = re.compile("set-reg (\w+) (\w+)")
                    match = expr.findall(_input)
                    if len(match) == 1:
                        values = match[0]
                        reg = int(values[0], 0)
                        # ^ Using a base of 0 auto-detects the number format.
                        # This allows for decoding of hex prefixes (0x45), binary (0b14),
                        # standard base 10, etc. automatically.
                        val = int(values[1], 0)
                        _input = f"set-reg {reg} {val}"
                    else:
                        _print(f"Failed to parse set-reg command: '{_input}'")
                        _input = ""

                elif "get-reg" in _input:
                    expr = re.compile("get-reg (\w+)")
                    match = expr.findall(_input)
                    if len(match) == 1:
                        # Since there is just a single value in the regex, 'match' will
                        # contain the register address.
                        reg = int(match[0], 0)
                        _input = f"get-reg {reg}"
                    else:
                        _print(f"Failed to parse get-reg command: '{_input}'")
                        _input = ""

                self.lock_input.acquire()
                self.input = _input
                self.lock_input.release()

                _print("")

        except Exception as e:
            print(traceback.format_exc())
            self.quit()

    """
    This blocking function manages the serial interface to the MCU.  The 
    serial interface will perform a blocking readline() if there is any
    input data to be read.  Therefore, the MCU should terminate any commands
    with a newline '\n' character.  The newline character will be stripped
    from the command string and processed in this function.

    It can be killed by setting the member "kill_serial" variable to True.
    It will also die (gracefully) if it throws any internal exceptions.
    """
    def run_serial_interface(self):
        try:
            while not self.kill_serial:

                # Process outgoing commands
                if not self.lock_input.locked() and self.input != "":
                    self.lock_input.acquire()

                    self.s.write(bytes(self.input, encoding="ascii") + b'\n')

                    # Clear the input
                    self.input = ""
                    self.lock_input.release()

                while(self.s.in_waiting):
                    # There is some serial port data to process
                    recvd = self.s.readline()

                    # This part is mostly for debugging.
                    # Attempt to parse the received line into an ascii string.
                    # If it fails, then we'll skip the string conversion and leave
                    # the received data as bytes.
                    # ---
                    command = False
                    try:
                        decoded = recvd.decode(encoding="ascii").strip()
                        recvd = decoded
                        command = True
                    except:
                        command = False
                        pass
                    # ---

                    _print(f"MCU: {recvd}") # Echo the received message
                    
                    if command:
                        # Process received command
                        if (recvd == "*SYNC*"):
                            self.s.write(b"*SYNC*")

                        elif ("*IMG*" in recvd):
                            # There is an incoming image.
                            
                            # First, we expect the MCU to tell us about the incoming
                            # image data.  It will send a 'header' string of the
                            # following format:
                            # "*IMG* [FORMAT] [LENGTH IN BYTES] [WIDTH] [HEIGHT]"
                            # Which corresponds to the regular expression below.
                            expr = re.compile("\*IMG\* (\w+) (\d+) (\d+) (\d+)")
                            match = expr.findall(recvd)

                            if len(match) == 1:
                                # Received expected header, parse parameters from regex
                                values = match[0]
                                pixel_format = values[0]
                                expected = int(values[1])
                                w = int(values[2])
                                h = int(values[3])

                                # Enter "receive" mode, where we'll wait for the expected
                                # number of bytes.
                                _print(f"Collecting {expected} bytes...")
                                img_raw = self.s.read(expected)
                                
                                if (len(img_raw) != expected):
                                    # Failed to receive expected number of bytes.
                                    _print(f"Image receive failed!  Only received {len(img_raw)}/{expected} bytes")
                                else:
                                    filename = "Image.png"
                                    convert(img_raw, filename, w, h, pixel_format)
                                    _print(f"Saved image to '{Path(filename).absolute()}'")

                                    # The code below will display the image in an OpenCV
                                    # window.  It's disabled by default because it won't
                                    # work on headless systems.  Uncomment to enable.
                                    # image = cv2.imread(filename)
                                    # cv2.imshow(" ", image)
                                    # cv2.waitKey(1)

        except Exception as e:
            print(traceback.format_exc())
            self.quit()

    def quit(self):
        self.kill_get_input = True
        self.kill_serial = True
        exit()

# Set up command-line arguments
parser = argparse.ArgumentParser()
parser.add_argument("port", type=str, help="Serial port to connect to")
parser.add_argument("--baudrate", type=int, default=921600, help="Baud rate (default 921600)")
parser.add_argument("--timeout", type=int, default=5, help="Communication timeout in seconds (default 5)")

if __name__ == "__main__":
    args = parser.parse_args()
    console = CameraIFConsole(port=args.port, baudrate=args.baudrate, timeout=args.timeout)
