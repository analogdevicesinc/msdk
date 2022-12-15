*** settings ***
Library    String
Library    ../resources/serialPortReader.py
Suite Setup        Open Ports    ${SERIAL_PORT_1}    ${SERIAL_PORT_2}
Suite Teardown     Close Ports 

*** Variables ***
${SERIAL_PORT_1}    /dev/ttyUSB0
${SERIAL_PORT_2}  /dev/ttyUSB1

*** test cases ***
Initial Connection Test Server
    [Timeout]     60s
    # inital sleep to allow device time to boot up after programming
    Read ALL     >>> Prompt user to enter passkey <<<    40    ${SERIAL_PORT_1}

Secured Connection Test Server
    [Timeout]     60s
    # inital sleep to allow device time to boot up after programming
    sleep    2
    Expect And Timeout    pin 1 1234\n    > smpSmExecute event=4 state=12    5    ${SERIAL_PORT_2}

Secured Connection Test Client
    [Timeout]     90s
    # inital sleep to allow device time to boot up after programming
    sleep    2
    Expect And Timeout    pin 1 1234\n    >>> Pairing completed successfully <<<    25    ${SERIAL_PORT_1}

Write Characteristic Test
    [Timeout]     30s
    sleep    2
    Expect And Timeout    btn 2 l\n     hello back     5    ${SERIAL_PORT_1}    

Write Secure Characteristic Test
    [Timeout]     30s
    sleep    2
    Expect And Timeout    btn 2 m\n    Notification from secure data service      5      ${SERIAL_PORT_1}    


Speed Test  
    [Timeout]     190s
    sleep    2
    Expect And Timeout Simple    btn 2 x\n    bits transferred in    40    ${SERIAL_PORT_1}


Phy Switching Test 
    [Timeout]     30s
    sleep    3
    Expect And Timeout    btn 2 s\n    DM_PHY_UPDATE_IND - RX: 2, TX: 2    5    ${SERIAL_PORT_1}