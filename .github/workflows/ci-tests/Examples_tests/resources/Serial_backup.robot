
*** Settings ***
Library    SerialLibrary      encoding=ascii
Library    String

*** Variables ***
${is_verbose}   ${VERBOSE}

*** Keywords ***
Open Serial Port
    [Arguments]    ${SERIAL_PORT_1}  
    
    Add Port   ${SERIAL_PORT_1}
    ...        baudrate=115200
    ...        bytesize=8
    ...        parity=N
    ...        stopbits=1
    ...        timeout=999
   
Close Serial Port
    Delete All Ports

Send
    [Arguments]    ${data}
    Write Data    ${data}

Expect And Timeout   
    [Arguments]    ${data}    ${timeout}
    [Timeout]    ${timeout}
    ${read} =     Read Until    ${data}
    Log Serial Traffic    ${read}
    
Expect And Timeout No Verbose  
    [Arguments]    ${data}    ${timeout}
    [Timeout]    ${timeout}
    ${read} =     Read Until    ${data}
   
   
Log Serial Traffic
    [Arguments]    ${data}
    Log To Console    \n
    Log To Console  ${data}
    Log To Console    \n
    
