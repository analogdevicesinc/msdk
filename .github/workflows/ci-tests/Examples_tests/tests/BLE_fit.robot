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
    Read All    Fit got evt 52    30    ${SERIAL_PORT_1}

Button S Press Test
    [Timeout]    30s
    Expect And Timeout    btn 1 s\n      Short Button 1 Press    5    ${SERIAL_PORT_1}

Button M Press Test
    [Timeout]    30s
    Expect And Timeout    btn 1 m\n      Medium Button 1 Press    5    ${SERIAL_PORT_1}
