
*** Settings ***
Library    SerialLibrary      encoding=ascii
Library    String

*** Variables ***
${is_verbose}   ${VERBOSE}
${PORT}    ${SERIAL_PORT_2}
*** Keywords ***
Open Serial Port 2
    [Arguments]    ${SERIAL_PORT_2}  
    
    Add Port   ${SERIAL_PORT_2}
    ...        baudrate=115200
    ...        bytesize=8
    ...        parity=N
    ...        stopbits=1
    ...        timeout=999
   
Close Serial Port 2
    Delete All Ports

Send
    [Arguments]    ${data}    
    Write Data    ${data}    NONE    NONE    ${PORT}

Expect And Timeout
    [Arguments]    ${data}    ${timeout}
    [Timeout]    ${timeout}
    ${read} =     Read Until    ${data}
    Log Serial Traffic 2    ${read}
    

Log Serial Traffic 2
    [Arguments]    ${data}

    Log To Console    \n
    Log To Console  ${data}
    Log To Console    \n

