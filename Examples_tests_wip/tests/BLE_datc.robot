*** settings ***
Library    SerialLibrary      encoding=ascii
Library    String
Resource     ../resources/Serial.robot
Suite Setup        Serial.Open Serial Port    ${SERIAL_PORT_1}    NONE    
Suite Teardown     Serial.Close Serial Port

*** Variables ***
${SERIAL_PORT_1}    /dev/ttyUSB0
${VERBOSE}     None

*** test cases ***
Stop Scanning Test
    [Timeout]    30s
    # inital sleep to allow device time to boot up after programming
    Sleep     5s
    Serial.Send    btn 1 s\n    ${SERIAL_PORT_1}   
    Serial.Expect And Timeout    >>> Scanning stopped <<<    10    ${SERIAL_PORT_1}

Button Press Test
    [Timeout]    30s
    Serial.Send    btn 1 m\n    ${SERIAL_PORT_1}
    Serial.Expect And Timeout    Medium Button 1 Press    5    ${SERIAL_PORT_1}

No button Action Test
    [Timeout]    30s
    Serial.Send    btn 2 s\n    ${SERIAL_PORT_1}
    Serial.Expect And Timeout    No action assigned    5    ${SERIAL_PORT_1}

Clearing Bond Info Test
    [Timeout]    30s
    Serial.Send    btn 1 l\n    ${SERIAL_PORT_1}
    Serial.Expect And Timeout    Clear bonding info    10    ${SERIAL_PORT_1}
