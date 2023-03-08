## Description

Simple serial port console for FCC testing.


## Software

### Project Usage

Universal instructions on building, flashing, and debugging this project can be found in the **[MSDK User Guide](https://analog-devices-msdk.github.io/msdk/USERGUIDE/)**.

### Project-Specific Build Notes

(None - this project builds as a standard example)

# Usage

## LEDs

The red LED will indicate that an error assertion has occurred.  

The green LED indicates CPU activity. When the LED is on, the CPU is active, when the LED
is off, the CPU is in sleep mode.

## Setup

### Board Selection

Before building firmware you must select the correct value for BOARD in [project.mk](project.mk), e.g. "EvKit_V1".

### Required Connections
-   Connect a USB cable between the PC and the (USB/PWR - UART) connector.

## Trace Serial Port
When TRACE is enabled in the [project.mk](project.mk), the on-board USB-to-UART adapter can
be used to view the trace messages and interact with the application. Open a serial port terminal with
the following settings.

Baud            : 115200  
Char size       : 8  
Parity          : None  
Stop bits       : 1  
HW Flow Control : No  
SW Flow Control : No  

### Expected Output


On startup:
```
    RAM: 4 x 752 bytes -- connection context
    RAM: 16 x 719 bytes -- Tx buffer descriptors
    RAM: 6 x 2296 bytes -- advertising set context
LlHandlerInit: LL initialization completed
    opModeFlags = 0x005F5C40
### LlApi ###  LlSetBdAddr
Static BDA[5:3]=00:18:80
       BDA[2:0]=06:81:0C
Usage:
 (0) Transmit on RF channel 0 (2402 MHz)
 (1) Transmit on RF channel 19 (2440 MHz)
 (2) Transmit on RF channel 39 (2480 MHz)
 (3) Receive  on RF channel 39 (2480 MHz)
 (4) Set Transmit power
 (5) Enable constant TX
 (6) Disable constant TX -- MUST be called after (5)
 (8) Set PHY
 (9) TX Frequency Hop
 (e) End transmission -- MUST be used after each (0-3, 9)
 (u) Print usage
```


On TX test start (0) and stop (e)
```
Transmit RF channel 0, 255 bytes/pkt, 0xAA, 1M PHY, forever ..
### LlApi ###  LlTxTest, rfChan=0, len=255, pktType=7
res = 0 (SUCCESS)
End test
### LlApi ###  LlEndTest
Test completed, numTx=1106
                numRxSuccess=0
                numRxCrcError=0
                numRxTimeout=0
```

On RX test start (3) and stop (e)
```
Receive RF channel 39, 1M PHY, forever ..                                                                              
### LlApi ###  LlRxTest, rfChan=39                                                                                     
res = 0 (SUCCESS)                                                                                                      
End test                                                                                                               
### LlApi ###  LlEndTest                                                                                               
Test completed, numTx=0                                                                                                
                numRxSuccess=658                                                                                       
                numRxCrcError=0                                                                                        
                numRxTimeout=0                                                                                         
```

On TX Frequency Hop test start (9) and stop (e)
```
Starting frequency hopping
### LlApi ###  LlTxTest, rfChan=0, len=255, pktType=7
### LlApi ###  LlTxTest, rfChan=1, len=255, pktType=7
...
### LlApi ###  LlTxTest, rfChan=23, len=255, pktType=7
### LlApi ###  LlTxTest, rfChan=24, len=255, pktType=7
End test
### LlApi ###  LlEndTest
Test completed, numTx=1304
                numRxSuccess=0
                numRxCrcError=0
                numRxTimeout=0
```

