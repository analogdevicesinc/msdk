import serial
import time
from robot.api import logger
from robot.libraries.BuiltIn import BuiltIn
def write_to_console(s):
    logger.console(s,newline=False)
def expect_and_timeout(send=None,expect=None, timeout= 10, port=None):
    timeStart = time.time()
    with serial.Serial() as ser:
        ser.baudrate = 115200
        ser.port = port
        ser.timeout=1
        ser.open()
        time.sleep(0.1)
        #flush junk
        ser.write(bytes("\n", encoding='utf-8'))
        ser.reset_input_buffer()
        ser.reset_output_buffer()
        time.sleep(0.1)
        ser.read_all()
        
        time.sleep(0.1)
        if send == None:
            send ="\r\n"
        write_to_console("\r\nWriting to port: " + port  + "\r\n")
        time.sleep(0.1)
        # start test, send command
        ser.write(bytes(send, encoding='utf-8'))
        while (time.time()-timeStart) < timeout:
            x=ser.readline().decode("utf-8")
            x=str(x)
            if x != "":
                write_to_console(x)
                if expect in x:
                    write_to_console("\r\n")
                    ser.close()
                    BuiltIn().pass_execution("Woohoo!\r\n")
        write_to_console("\r\nTest Timeout trying one more time")
        ser.write(bytes("\n", encoding='utf-8'))
        time.sleep(0.1)
        ser.write(bytes(send, encoding='utf-8'))
        timeStart = time.time()
        while (time.time()-timeStart) < timeout:
            x=ser.readline().decode("utf-8")
            x=str(x)
            if x != "":
                write_to_console(x)
                if expect in x:
                    write_to_console("\r\n")
                    ser.close()
                    BuiltIn().pass_execution("Woohoo!\r\n")

        ser.close()
        BuiltIn().fail("Darn!\r\n")


def flush_junk( port=None):

    with serial.Serial() as ser:
        ser.baudrate = 115200
        ser.port = port
        ser.timeout=1
        ser.open()
        # serial tends to return before port is open
        time.sleep(0.1)
        # write something and read it back to get any junk data from start up
        ser.write(bytes("\n", encoding='utf-8'))
        # flushing does not seem to help
        ser.reset_input_buffer()
        ser.reset_output_buffer()
        time.sleep(0.1)
        ser.read_all()
        ser.close()