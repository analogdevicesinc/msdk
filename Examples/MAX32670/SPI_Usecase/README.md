## Description

This example will send some test byte from master (SPI0) to slave (SPI1) then from slave to master
To change number of bytes please change TEST_BUFF_SIZE in spi_config.h file
Please connect SPI0 pins to SPI1 on EvKit
    
## Required Connections

-   Connect a USB cable between the PC and the CN1 (USB/PWR) connector.
-   Connect SPI0 pins with SPI1 with jumpers
        MISO: P0.2 <--> P0.14
        MOSI: P0.3 <--> P0.15
        CLK : P0.4 <--> P0.16
        SS  : P0.5 <--> P0.17
-   Open an terminal application on the PC and connect to the EV kit's console UART at 115200, 8-N-1.

## Expected Output

The Console UART of the device will output these messages:

```
-----------------------------------------------------------------------------------------------
SPI0 is configured as master
SPI1 is configured as slave
Please use jumper to connect these two spi ports:
        MISO: P0.2 <--> P0.14
        MOSI: P0.3 <--> P0.15
        CLK : P0.4 <--> P0.16
        SS  : P0.5 <--> P0.17

This example will send some test byte from master to slave then from slave to master
To change number of bytes please change TEST_BUFF_SIZE
-----------------------------------------------------------------------------------------------

Master Send Packet
01 02 03 04 05 06 07 08 09 0A 0B 0C 0D 0E 0F 10
11 12 13 14 15 16 17 18 19 1A 1B 1C 1D 1E 1F 20
21 22 23 24 25 26 27 28 29 2A 2B 2C 2D 2E 2F 30
31 32 33 34 35 36 37 38 39 3A 3B 3C 3D 3E 3F 40

Master Expect Packet
FF FE FD FC FB FA F9 F8 F7 F6 F5 F4 F3 F2 F1 F0
EF EE ED EC EB EA E9 E8 E7 E6 E5 E4 E3 E2 E1 E0
DF DE DD DC DB DA D9 D8 D7 D6 D5 D4 D3 D2 D1 D0
CF CE CD CC CB CA C9 C8 C7 C6 C5 C4 C3 C2 C1 C0
-------------------------------------------------------

Master Receive:
FF FE FD FC FB FA F9 F8 F7 F6 F5 F4 F3 F2 F1 F0
EF EE ED EC EB EA E9 E8 E7 E6 E5 E4 E3 E2 E1 E0
DF DE DD DC DB DA D9 D8 D7 D6 D5 D4 D3 D2 D1 D0
CF CE CD CC CB CA C9 C8 C7 C6 C5 C4 C3 C2 C1 C0

Master Send Receive Succeeded

Slave Receive:
01 02 03 04 05 06 07 08 09 0A 0B 0C 0D 0E 0F 10
11 12 13 14 15 16 17 18 19 1A 1B 1C 1D 1E 1F 20
21 22 23 24 25 26 27 28 29 2A 2B 2C 2D 2E 2F 30
31 32 33 34 35 36 37 38 39 3A 3B 3C 3D 3E 3F 40

Slave Send Receive Succeeded

End of Example!
```

