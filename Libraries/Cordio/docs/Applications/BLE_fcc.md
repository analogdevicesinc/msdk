# BLE_fcc

Project builds a simple serial port console for FCC testing.

# Usage

## LEDs

The red LED will indicate that an error assertion has occurred.  

The green LED indicates CPU activity. When the LED is on, the CPU is active, when the LED
is off, the CPU is in sleep mode.

## Expected Output


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

