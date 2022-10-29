*** settings ***
Library    SerialLibrary      encoding=ascii
Library    String
Resource     ../resources/Serial.robot
Suite Setup        Serial.Open Serial Port    ${SERIAL_PORT_1}    NONE    
Suite Teardown     Serial.Close Serial Port

*** Variables ***
${SERIAL_PORT_1}    None
${VERBOSE}     None

*** test cases ***
Stop Scanning Test
    [Timeout]    30s
    # inital sleep to allow device time to boot up after programming
    Sleep    5s
    Serial.Send    btn 1 s\n    ${SERIAL_PORT_1}
    Serial.Expect And Timeout     >>> Scanning stopped <<<    5    ${SERIAL_PORT_1}

Connection ID Test
    [Timeout]    30s
    Serial.Send    btn 1 m\n    ${SERIAL_PORT_1}
    Serial.Expect and timeout    ConnID for Button Press:    5    ${SERIAL_PORT_1}

Clear Resolving List Test
    [Timeout]    30s
    Serial.Send    btn 1 l\n    ${SERIAL_PORT_1}
    serial.Expect And Timeout    Clear resolving list status 0x00    5    ${SERIAL_PORT_1}