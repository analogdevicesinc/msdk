import serial
import time
from robot.api import logger
from robot.libraries.BuiltIn import BuiltIn

# stores open ports for later use
used_ports=dict()
#--------------------------------------------------------------------------------------
def write_to_console(s, log=True):
    if log == True:
        logger.console("(robot)> "+ s ,newline=True)
    else:
        logger.console(s,newline=False)
#--------------------------------------------------------------------------------------
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
            write_to_console("Port 2 was not open to begin with \r\n")
#--------------------------------------------------------------------------------------
def expect_and_timeout(send=None,expect=None, timeout= 10, port=None):
    attempt_count=0
    x=""
    #decide which port to use for this instance of the method call
    while True:
        attempt_count+=1
        timeStart = time.time()
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
                # start test, send command
                used_ports[port].write(bytes(send, encoding='utf-8'))
                time.sleep(0.2)
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
def read_all(expect=None,timeout=10,port=None):
    global used_ports
    timeStart = time.time()
    attempt_count=0
    while True:
        attempt_count+=1
        timeStart = time.time()   
        while used_ports[port].is_open != True:
            time.sleep(0.1)
        if used_ports[port].is_open == True:
            while (time.time()-timeStart) < timeout:
                x=used_ports[port].readline().decode("utf-8")
                x=str(x)
                if x != "":
                    write_to_console(x,False)
                    if expect in x:                       
                        BuiltIn().pass_execution(".")
            # timed out
            break
    BuiltIn().fail(" .\r\n")
#--------------------------------------------------------------------------------------
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