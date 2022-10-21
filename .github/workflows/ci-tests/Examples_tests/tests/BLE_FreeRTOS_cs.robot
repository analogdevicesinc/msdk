*** settings ***
Library    SerialLibrary      encoding=ascii
Library    String
Resource     ../resources/Serial.robot
Suite Setup        Serial.Open Serial Port    ${SERIAL_PORT_1}    NONE    
Suite Teardown     Serial.Close Serial Port

*** Variables ***
${SERIAL_PORT_1}    /dev/ttyUSB0
${VERBOSE}    0

*** test cases ***
Write Characteristic Test
    [Timeout]     30s
    # inital sleep to allow device time to boot up after programming
    sleep    5
    Serial.send    btn 2 l\n    ${SERIAL_PORT_1}
    Serial.Expect And Timeout   hello back    2s    ${SERIAL_PORT_1}


# Phy Switching Test 
#     [Timeout]     30s
#     sleep     2
#     Serial.send    btn 2 s\n
#     Serial.Expect And Timeout    DM_PHY_UPDATE_IND - RX: 2, TX: 2    10s


# Speed Test  
#     [Timeout]     60s
#     Serial.send    btn 2 x\n
#     Serial.Expect And Timeout    bits transferred in    30s

