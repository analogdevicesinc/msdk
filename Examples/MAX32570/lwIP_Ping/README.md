## Description

This example demostrate basic ping feature over lightweight IP stack, the example will ping 192.168.100.1
Instead of using static IP, dynamic ip can be used, 
Update "USE_DHCP" definition in the C:/MaximSDK/Libraries/lwIP/include/Maxim/lwipcfg.h" file
to get dynamic IP.
Static ip can be updated in lwipcfg.h file too.

## Required Connections

-   Connect a USB cable between the PC and the CN1 (USB/PWR) connector.
-   Connect a USB cable between the PC and the CN1 (USB/UART0) connector.
-   Open an terminal application on the PC and connect to the EV kit's console UART at 115200, 8-N-1.
-   Connect a Ethernet cable between EvKit and PC or Modem.

The ping messages can be view over whireshark or other network protocol analayzer.

## Expected Output
```
*** Ping Example ***
Link Status: Up
ping: send 192.168.100.1
ping: send 192.168.100.1
ping: send 192.168.100.1
ping: send 192.168.100.1
ping: send 192.168.100.1
ping: send 192.168.100.1
ping: send 192.168.100.1
ping: send 192.168.100.1
ping: send 192.168.100.1
ping: send 192.168.100.1
ping: send 192.168.100.1
ping: send 192.168.100.1
ping: send 192.168.100.1
ping: send 192.168.100.1
```
