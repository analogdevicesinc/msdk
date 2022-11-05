*** settings ***
Library    String
Library    ../resources/serialPortReader.py

*** Variables ***
${SERIAL_PORT_1}    None


*** test cases ***
Button S Press Test
[   Timeout]    30s
    Sleep     5s
    Expect And Timeout    btn 1 s\n      Short Button 1 Press    5    ${SERIAL_PORT_1}

Button M Press Test
    [Timeout]    30s
    Expect And Timeout    btn 1 m\n      Medium Button 1 Press    5    ${SERIAL_PORT_1}
