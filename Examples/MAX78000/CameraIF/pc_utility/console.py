"""
This file implements a simple console program for communicating
with the 'CameraIF' firmware.

Installation:
1. Install Python 3
2. pip install -r requirements.txt

Run "python console.py -h" for help.

"""

from serial import Serial
from threading import Thread, Lock
import signal
import re
from imgConverter import convert
import argparse
import cv2

class CameraIFConsole():
    def __init__(self, port, baudrate=921600, timeout=5):
        self.thr_get_input = Thread(target=self.get_input)
        self.kill_get_input = False
        self.thr_serial = Thread(target=self.run_serial_interface)
        self.kill_serial = False

        self.input = ""
        self.lock_input = Lock()

        self.s = Serial(port=port, baudrate=baudrate, timeout=timeout)

        self.thr_get_input.start()
        self.thr_serial.start()
        print("Started CameraIF console.  Type 'help' for help, 'quit' to quit.")
        

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
                elif _input == "help" or _input == "h":
                    self.help()
                else:
                    self.lock_input.acquire()
                    self.input = _input
                    self.lock_input.release()

        except Exception as e:
            print(e)
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

                    print(f"MCU: {recvd}") # Echo the received message
                    
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
                                length = int(values[1])
                                w = int(values[2])
                                h = int(values[3])

                                # Enter "receive" mode, where we'll wait for the expected
                                # number of bytes.
                                print(f"Waiting for {length} bytes...")
                                img_raw = self.s.read(length)
                                
                                if (len(img_raw) != length):
                                    # Failed to receive expected number of bytes.
                                    print(f"Image receive failed!  Only received {len(img_raw)/length} bytes")
                                else:
                                    convert(img_raw, "Image.png", w, h, pixel_format)
                                    image = cv2.imread("image.png")
                                    cv2.imshow(" ", image)
                                    cv2.waitKey(1)

        except Exception as e:
            print(e)
            self.quit()

    def quit(self):
        self.kill_get_input = True
        self.kill_serial = True
        exit()

    def help(self):
        print("Available commands:")
        print("\t'quit' : Quits this console")
        print("\t'help' : Prints this help string")
        print("\t'capture' : This command will perform a blocking capture of a single image.")
        print("\t'imgres' <width> <height> : Set the image resolution of the camera to <width> x <height>")
        print("\t'enable_dma' : Enables DMA for standard blocking captures ('capture')")
        print("\t'disable_dma' : Disables DMA for standard blocking captures ('capture')")
        print("\t'stream' : Performs a line-by-line streaming DMA capture of a single image.")

# Set up command-line arguments
parser = argparse.ArgumentParser()
parser.add_argument("port", type=str, help="Serial port to connect to")
parser.add_argument("--baudrate", type=int, default=921600, help="Baud rate (default 921600)")
parser.add_argument("--timeout", type=int, default=5, help="Communication timeout in seconds (default 5)")

if __name__ == "__main__":
    args = parser.parse_args()
    console = CameraIFConsole(port=args.port, baudrate=args.baudrate, timeout=args.timeout)
