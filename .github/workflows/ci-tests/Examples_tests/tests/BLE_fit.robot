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
Button Press Tests
    [Timeout]    30s
    # inital sleep to allow device time to boot up after programming
    Sleep     5s
    Serial.Send    btn 1 s\n    ${SERIAL_PORT_1}
    Serial.Expect And Timeout    Short Button 1 Press    5s    ${SERIAL_PORT_1}

    Serial.Send    btn 1 m\n    ${SERIAL_PORT_1}
    Serial.Expect And Timeout    Medium Button 1 Press    5s    ${SERIAL_PORT_1}
