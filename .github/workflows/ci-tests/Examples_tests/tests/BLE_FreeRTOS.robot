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
Stop Advertising Test
    [Timeout]    30s
    # inital sleep to allow device time to boot up after programming
    sleep    5
    Serial.Send    btn 2 s\n    ${SERIAL_PORT_1}    
    Serial.Expect And Timeout    >>> Advertising stopped <<<    10s    ${SERIAL_PORT_1}


Start Advertising Test  
    [Timeout]     30s
    Serial.send    btn 1 s\n    ${SERIAL_PORT_1}
    Serial.Expect And Timeout    >>> Advertising started <<<    10s    ${SERIAL_PORT_1}


Button M Press Test  
    [Timeout]     30s
    Serial.send    btn 1 m\n    ${SERIAL_PORT_1}
    Serial.Expect And Timeout    Medium Button 1 Press    5s    ${SERIAL_PORT_1}
    

Button L Press Test  
    [Timeout]     30s
    Serial.send    btn 1 l\n    ${SERIAL_PORT_1}
    Serial.Expect And Timeout    Clear resolving list status 0x00    5S    ${SERIAL_PORT_1}
   

Button X Press Test  
    [Timeout]     30s
    Serial.send    btn 1 x\n    ${SERIAL_PORT_1}
    Serial.Expect And Timeout    Stack Version: Packetcraft Host    5S    ${SERIAL_PORT_1}