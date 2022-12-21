*** settings ***
Library    String
Library    ../resources/serialPortReader.py
Suite Setup        Open Ports    ${SERIAL_PORT_1}
Suite Teardown     Close Ports 

*** Variables ***
${SERIAL_PORT_1}    None


*** test cases ***
Stop Advertising Test
    [Timeout]    30s
    # inital sleep to allow device time to boot up after programming
    sleep    5
    Expect And Timeout    btn 2 s\n      >>> Advertising stopped <<<    10    ${SERIAL_PORT_1}


Start Advertising Test  
    [Timeout]     30s
    Expect And Timeout    btn 1 s\n      >>> Advertising started <<<    10    ${SERIAL_PORT_1}


Button M Press Test  
    [Timeout]     30s
    Expect And Timeout    btn 1 m\n      Medium Button 1 Press    5    ${SERIAL_PORT_1}
    

Button L Press Test  
    [Timeout]     30s
    Expect And Timeout    btn 1 l\n      Clear resolving list status 0x00    5    ${SERIAL_PORT_1}
   

Button X Press Test  
    [Timeout]     30s
    Expect And Timeout    btn 1 x\n      Stack Version: Packetcraft Host    5    ${SERIAL_PORT_1}
   




    