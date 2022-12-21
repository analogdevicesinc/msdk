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
    Read All     Advertising started     30    ${SERIAL_PORT_1}

Button S Press Test
    [Timeout]    30s
    # inital sleep to allow device time to boot up after programming
    Expect And Timeout    btn 1 s\n      mcsAppBtnCback; 2    5    ${SERIAL_PORT_1}

Button M Press Test
    [Timeout]    30s
    Expect And Timeout    btn 1 m\n      mcsAppBtnCback; 3    5    ${SERIAL_PORT_1}
    
Button L Press Test
    [Timeout]    30s
    Expect And Timeout    btn 1 l\n      mcsAppBtnCback; 4    5    ${SERIAL_PORT_1}