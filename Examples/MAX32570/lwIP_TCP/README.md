## Description

This example demostrate basic tcp echo server by using lightweight IP stack, 
The example IP is 192.168.100.200 it is defined in C:/MaximSDK/Libraries/lwIP/include/Maxim/lwipcfg.h" file
TCP echo server port is 7

Instead of using static IP, dynamic ip can be used, 
Update "USE_DHCP" definition in the C:/MaximSDK/Libraries/lwIP/include/Maxim/lwipcfg.h" file to get dynamic IP.


## Required Connections

-   Connect a USB cable between the PC and the CN1 (USB/PWR) connector.
-   Connect a USB cable between the PC and the CN1 (USB/UART0) connector.
-   Open an terminal application on the PC and connect to the EV kit's console UART at 115200, 8-N-1.
-   Connect a Ethernet cable between EvKit and PC or Modem.
-   Run a tcp client
-   Connect to IP:192.168.100.200 Port:7
	Any tcp utilies can be used, as an example hercules might be a good option
    Link: https://www.hw-group.com/software/hercules-setup-utility
-   Send messages to tcp server

## Expected Output
```
*** TCP Echo Server Example ***
Link Status: Up

```



