import serial
import time
from robot.api import logger
from robot.libraries.BuiltIn import BuiltIn

# stores open ports for later use
used_ports=dict()
#--------------------------------------------------------------------------------------
'''
Prints the output form serial port as well as debugging messages
this switch is used to help differentiate by add "(robot)" to debugging messages
'''
def write_to_console(s, log=True):
    if log == True:
        logger.console("(robot)> "+ s ,newline=True)
    else:
        logger.console(s,newline=False)
#--------------------------------------------------------------------------------------
'''
Opens up to 2 ports used during tests,
stores in global dictionary for late use
This is only called once upon test setup
'''
def open_ports(port_1=None , port_2=None):
    global used_ports
    if port_1 != None :
        
        write_to_console(f"Trying to open port: {port_1}",True) 
        used_ports[port_1] = serial.Serial(port=port_1,baudrate=115200, timeout=0)
        while used_ports[port_1].is_open != True:
            time.sleep(0.1)
        write_to_console(f"Serial Port {port_1} Opened Succesfully",True)
        
    if port_2!= None :
        write_to_console(f"Trying to open port: {port_2}" , True)
        used_ports[port_2] = serial.Serial(port=port_2,baudrate=115200, timeout=0)
        while used_ports[port_2].is_open != True:
            time.sleep(0.1)
        write_to_console(f"Serial Port {port_2} Opened Succesfully",True)
#--------------------------------------------------------------------------------------
'''
Closes instances of ope ports found in global dicitonary
This is only called once dudring test teardown
'''
def close_ports():
    global used_ports
    for port in used_ports.values():
        try:
            if isinstance(port, serial.Serial) == True:
                port.close()
                while port.is_open == True:
                    time.sleep(0.1)
                write_to_console("Serial Port Closed Succesfully\r\n")

        except Exception as err:
            write_to_console("Port was not open to begin with \r\n")
#--------------------------------------------------------------------------------------
'''
Can optioanlly send a string through serial port
and expect a string in return on  a single port
if timeout is exceeded the test is attempted once more
before failing.
'''
def expect_and_timeout(send=None,expect=None, timeout= 10, port=None):
    attempt_count=0
    x=""
    #decide which port to use for this instance of the method call
    while True:
        attempt_count+=1
        timeStart = time.time()
        char_list=[]
        while used_ports[port].is_open != True:
            time.sleep(0.1)
        if used_ports[port].is_open == True:
            #flush junk
            used_ports[port].reset_input_buffer()
            used_ports[port].reset_output_buffer()
            time.sleep(0.1)
            used_ports[port].write(bytes("\n", encoding='utf-8'))
            # send data if any
            if send != None:
                time.sleep(0.1)
                char_list = list(send)
                for char in char_list:
                # start test, send command
                    time.sleep(0.1)
                    used_ports[port].write(bytes(char, encoding='utf-8'))
            # read lines
            while (time.time()-timeStart) < timeout:
                try:
                    x=used_ports[port].read(10000).decode("utf-8",'ignore')
                except Exception as err:
                    pass
                x=str(x)
                if x != "":
                    write_to_console(x,False)
                if str(expect) in x:
                    BuiltIn().pass_execution(".")

            if attempt_count == 2:
                break
    write_to_console("\r\n-------\r\n",False)
    BuiltIn().fail(".")
#--------------------------------------------------------------------------------------
'''
Reads all incoming serial traffic, inclduing
what is in the buffer, for some given time
No multiple attemps and also expects a string to
eventually be read, or else timeouts
'''
def read_all(expect=None,timeout=10,port=None):
    global used_ports
    timeStart = time.time()
    while True:
        timeStart = time.time()   
        while used_ports[port].is_open != True:
            time.sleep(0.1)
        if used_ports[port].is_open == True:
            while (time.time()-timeStart) < timeout:
                x=used_ports[port].readline().decode("utf-8", 'ignore')
                x=str(x)
                if x != "":
                    write_to_console(x,False)
                    if expect in x:                       
                        BuiltIn().pass_execution(".")
            # timed out
            break
    BuiltIn().fail(" .\r\n")
