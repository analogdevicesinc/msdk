*** settings ***
Library    String
Library    ../resources/serialPortReader.py
Suite Setup        Open Ports    ${SERIAL_PORT_1}
Suite Teardown     Close Ports 

*** Variables ***
${SERIAL_PORT_1}    None

*** test cases ***
Initial Connection Test
    [Timeout]    30s
    Read All     Scanning started     30    ${SERIAL_PORT_1}

Stop Scanning Test
    [Timeout]    30s
    Expect And Timeout     btn 1 s\n     >>> Scanning stopped <<<    5    ${SERIAL_PORT_1}

Connection ID Test
    [Timeout]    30s
    Expect and timeout    btn 1 m\n      ConnID for Button Press:    5    ${SERIAL_PORT_1}

Clear Resolving List Test
    [Timeout]    30s
    Expect And Timeout    btn 1 l\n      Clear resolving list status 0x00    5    ${SERIAL_PORT_1}