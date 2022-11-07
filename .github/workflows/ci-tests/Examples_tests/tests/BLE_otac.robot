*** settings ***
Library    String
Library    ../resources/serialPortReader.py

*** Variables ***
${SERIAL_PORT_1}    None

*** test cases ***
Stop Scanning Test
    [Timeout]    30s
    # inital sleep to allow device time to boot up after programming
    Sleep    5s
    Expect And Timeout     btn 1 s\n     >>> Scanning stopped <<<    5    ${SERIAL_PORT_1}

Connection ID Test
    [Timeout]    30s
    Expect and timeout    btn 1 m\n      ConnID for Button Press:    5    ${SERIAL_PORT_1}

Clear Resolving List Test
    [Timeout]    30s
    Expect And Timeout    btn 1 l\n      Clear resolving list status 0x00    5    ${SERIAL_PORT_1}