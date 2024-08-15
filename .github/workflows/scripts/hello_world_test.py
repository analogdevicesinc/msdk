import serial
import argparse
import sys
import time
from resource_manager import ResourceManager

if __name__== "__main__":
    parser = argparse.ArgumentParser(description='Script to verif Hello world output')
    parser.add_argument('board', help='Board to test Hello World on')
    args = parser.parse_args()

    board = args.board
    rman = ResourceManager()



    port = rman.get_item_value(f'{board}.console_port')
    port = serial.Serial(port)

    rman.resource_reset(board)
    time.sleep(5)

    text = port.read_all().decode('utf-8')


    text_lower = text.lower()
    if 'count' in text_lower and "hello" in text_lower:
        sys.exit(0)
    else:
        sys.exit(-1)
