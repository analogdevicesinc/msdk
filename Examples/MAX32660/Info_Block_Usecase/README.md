## Description

MAX32660 has 4KB information block which is between 0x00040000 to 0x00041000.
First 1KB of block is used to keep device related configuration
The reset 3KB can be used to store some data.
This example demonstrate how can you read/write information block.

To run write test you need to set WITH_WRITE_TEST flag in main.c file

For more information please check MAX32660 User Guide


## Software

### Project Usage

Universal instructions on building, flashing, and debugging this project can be found in the **[MSDK User Guide](https://analog-devices-msdk.github.io/msdk/USERGUIDE/)**.

### Project-Specific Build Notes

(None - this project builds as a standard example)

## Required Connections

-   Connect a USB cable between the PC and the CN2 (USB/PWR) connector.
-   Open an terminal application on the PC and connect to the EV kit's console UART at 115200, 8-N-1.

## Expected Output

The Console UART of the device will output these messages:

```
***** Info Memory Read/Write Example *****
***** This example demonstrates how you can read/write Info memory *****

***** MAXIM AREA *****

0x00040000:    0400134b 0e801241    ffffffaa ffffff97
0x00040010:    0000000b 000045d0    5a5aa5a5 5a5aa5a5
0x00040020:    5a5aa5a5 5a5aa5a5    5a5aa5a5 5a5aa5a5
0x00040030:    ffffffff ffffffff    ffffffff ffffffff
0x00040040:    ffffffff ffffffff    ffffffff ffffffff
0x00040050:    ffffffff ffffffff    ffffffff ffffffff
0x00040060:    ffffffff ffffffff    ffffffff ffffffff
0x00040070:    ffffffff ffffffff    ffffffff ffffffff
0x00040080:    ffffffff ffffffff    ffffffff ffffffff
0x00040090:    ffffffff ffffffff    ffffffff ffffffff
0x000400a0:    ffffffff ffffffff    ffffffff ffffffff
0x000400b0:    ffffffff ffffffff    ffffffff ffffffff
0x000400c0:    ffffffff ffffffff    ffffffff ffffffff
0x000400d0:    ffffffff ffffffff    ffffffff ffffffff
0x000400e0:    ffffffff ffffffff    ffffffff ffffffff
0x000400f0:    ffffffff ffffffff    ffffffff ffffffff
0x00040100:    ffffffff ffffffff    ffffffff ffffffff
0x00040110:    ffffffff ffffffff    ffffffff ffffffff
0x00040120:    ffffffff ffffffff    ffffffff ffffffff
0x00040130:    ffffffff ffffffff    ffffffff ffffffff
0x00040140:    ffffffff ffffffff    ffffffff ffffffff
0x00040150:    ffffffff ffffffff    ffffffff ffffffff
0x00040160:    ffffffff ffffffff    ffffffff ffffffff
0x00040170:    ffffffff ffffffff    ffffffff ffffffff
0x00040180:    00030008 21a20000    00000000 6da10003
0x00040190:    00000000 00000000    ffffffff c0e200ff
0x000401a0:    00075000 08fd0200    00000000 00000000
0x000401b0:    00000000 00000000    c16fff84 8f7cffea
0x000401c0:    fffbfff1 8d5fffff    fb643548 a26bffff
0x000401d0:    00000303 02c10000    02190291 7a520000
0x000401e0:    ffffffff ffffffff    ffffffff ffffffff
0x000401f0:    000055aa 000055aa    ffffffff ffffffff
0x00040200:    ffffffff ffffffff    ffffffff ffffffff
0x00040210:    ffffffff ffffffff    ffffffff ffffffff
0x00040220:    ffffffff ffffffff    ffffffff ffffffff
0x00040230:    ffffffff ffffffff    ffffffff ffffffff
0x00040240:    ffffffff ffffffff    ffffffff ffffffff
0x00040250:    ffffffff ffffffff    ffffffff ffffffff
0x00040260:    ffffffff ffffffff    ffffffff ffffffff
0x00040270:    ffffffff ffffffff    ffffffff ffffffff
0x00040280:    ffffffff ffffffff    ffffffff ffffffff
0x00040290:    ffffffff ffffffff    ffffffff ffffffff
0x000402a0:    ffffffff ffffffff    ffffffff ffffffff
0x000402b0:    ffffffff ffffffff    ffffffff ffffffff
0x000402c0:    ffffffff ffffffff    ffffffff ffffffff
0x000402d0:    ffffffff ffffffff    ffffffff ffffffff
0x000402e0:    ffffffff ffffffff    ffffffff ffffffff
0x000402f0:    ffffffff ffffffff    ffffffff ffffffff
0x00040300:    cf4f48a4 c4388c55    f313ac6b 820b31a4
0x00040310:    352ff7ef 11c6ba4f    9736557f 5e4da263
0x00040320:    e37ce125 c78b7745    c911c9b8 6b0d7461
0x00040330:    4ad11b75 6edb8669    ffffffff ffffffff
0x00040340:    ffffffff ffffffff    ffffffff ffffffff
0x00040350:    ffffffff ffffffff    ffffffff ffffffff
0x00040360:    ffffffff ffffffff    ffffffff ffffffff
0x00040370:    ffffffff ffffffff    ffffffff ffffffff
0x00040380:    ffffffff ffffffff    ffffffff ffffffff
0x00040390:    ffffffff ffffffff    ffffffff ffffffff
0x000403a0:    ffffffff ffffffff    ffffffff ffffffff
0x000403b0:    ffffffff ffffffff    ffffffff ffffffff
0x000403c0:    ffffffff ffffffff    ffffffff ffffffff
0x000403d0:    ffffffff ffffffff    ffffffff ffffffff
0x000403e0:    ffffffff ffffffff    ffffffff ffffffff
0x000403f0:    ffffffff ffffffff    ffffffff ffffffff

***** USER AREA *****

0x00040400:    ffffffff ffffffff    ffffffff ffffffff
0x00040410:    ffffffff ffffffff    ffffffff ffffffff
0x00040420:    ffffffff ffffffff    ffffffff ffffffff
0x00040430:    ffffffff ffffffff    ffffffff ffffffff
0x00040440:    ffffffff ffffffff    ffffffff ffffffff
0x00040450:    ffffffff ffffffff    ffffffff ffffffff
0x00040460:    ffffffff ffffffff    ffffffff ffffffff
0x00040470:    ffffffff ffffffff    ffffffff ffffffff
0x00040480:    ffffffff ffffffff    ffffffff ffffffff
0x00040490:    ffffffff ffffffff    ffffffff ffffffff
0x000404a0:    ffffffff ffffffff    ffffffff ffffffff
0x000404b0:    ffffffff ffffffff    ffffffff ffffffff
0x000404c0:    ffffffff ffffffff    ffffffff ffffffff
0x000404d0:    ffffffff ffffffff    ffffffff ffffffff
0x000404e0:    ffffffff ffffffff    ffffffff ffffffff
0x000404f0:    ffffffff ffffffff    ffffffff ffffffff
0x00040500:    ffffffff ffffffff    ffffffff ffffffff
0x00040510:    ffffffff ffffffff    ffffffff ffffffff
0x00040520:    ffffffff ffffffff    ffffffff ffffffff
0x00040530:    ffffffff ffffffff    ffffffff ffffffff
0x00040540:    ffffffff ffffffff    ffffffff ffffffff
0x00040550:    ffffffff ffffffff    ffffffff ffffffff
0x00040560:    ffffffff ffffffff    ffffffff ffffffff
0x00040570:    ffffffff ffffffff    ffffffff ffffffff
0x00040580:    ffffffff ffffffff    ffffffff ffffffff
0x00040590:    ffffffff ffffffff    ffffffff ffffffff
0x000405a0:    ffffffff ffffffff    ffffffff ffffffff
0x000405b0:    ffffffff ffffffff    ffffffff ffffffff
0x000405c0:    ffffffff ffffffff    ffffffff ffffffff
0x000405d0:    ffffffff ffffffff    ffffffff ffffffff
0x000405e0:    ffffffff ffffffff    ffffffff ffffffff
0x000405f0:    ffffffff ffffffff    ffffffff ffffffff
0x00040600:    ffffffff ffffffff    ffffffff ffffffff
0x00040610:    ffffffff ffffffff    ffffffff ffffffff
0x00040620:    ffffffff ffffffff    ffffffff ffffffff
0x00040630:    ffffffff ffffffff    ffffffff ffffffff
0x00040640:    ffffffff ffffffff    ffffffff ffffffff
0x00040650:    ffffffff ffffffff    ffffffff ffffffff
0x00040660:    ffffffff ffffffff    ffffffff ffffffff
0x00040670:    ffffffff ffffffff    ffffffff ffffffff
0x00040680:    ffffffff ffffffff    ffffffff ffffffff
0x00040690:    ffffffff ffffffff    ffffffff ffffffff
0x000406a0:    ffffffff ffffffff    ffffffff ffffffff
0x000406b0:    ffffffff ffffffff    ffffffff ffffffff
0x000406c0:    ffffffff ffffffff    ffffffff ffffffff
0x000406d0:    ffffffff ffffffff    ffffffff ffffffff
0x000406e0:    ffffffff ffffffff    ffffffff ffffffff
0x000406f0:    ffffffff ffffffff    ffffffff ffffffff
0x00040700:    ffffffff ffffffff    ffffffff ffffffff
0x00040710:    ffffffff ffffffff    ffffffff ffffffff
0x00040720:    ffffffff ffffffff    ffffffff ffffffff
0x00040730:    ffffffff ffffffff    ffffffff ffffffff
0x00040740:    ffffffff ffffffff    ffffffff ffffffff
0x00040750:    ffffffff ffffffff    ffffffff ffffffff
0x00040760:    ffffffff ffffffff    ffffffff ffffffff
0x00040770:    ffffffff ffffffff    ffffffff ffffffff
0x00040780:    ffffffff ffffffff    ffffffff ffffffff
0x00040790:    ffffffff ffffffff    ffffffff ffffffff
0x000407a0:    ffffffff ffffffff    ffffffff ffffffff
0x000407b0:    ffffffff ffffffff    ffffffff ffffffff
0x000407c0:    ffffffff ffffffff    ffffffff ffffffff
0x000407d0:    ffffffff ffffffff    ffffffff ffffffff
0x000407e0:    ffffffff ffffffff    ffffffff ffffffff
0x000407f0:    ffffffff ffffffff    ffffffff ffffffff
0x00040800:    ffffffff ffffffff    ffffffff ffffffff
0x00040810:    ffffffff ffffffff    ffffffff ffffffff
0x00040820:    ffffffff ffffffff    ffffffff ffffffff
0x00040830:    ffffffff ffffffff    ffffffff ffffffff
0x00040840:    ffffffff ffffffff    ffffffff ffffffff
0x00040850:    ffffffff ffffffff    ffffffff ffffffff
0x00040860:    ffffffff ffffffff    ffffffff ffffffff
0x00040870:    ffffffff ffffffff    ffffffff ffffffff
0x00040880:    ffffffff ffffffff    ffffffff ffffffff
0x00040890:    ffffffff ffffffff    ffffffff ffffffff
0x000408a0:    ffffffff ffffffff    ffffffff ffffffff
0x000408b0:    ffffffff ffffffff    ffffffff ffffffff
0x000408c0:    ffffffff ffffffff    ffffffff ffffffff
0x000408d0:    ffffffff ffffffff    ffffffff ffffffff
0x000408e0:    ffffffff ffffffff    ffffffff ffffffff
0x000408f0:    ffffffff ffffffff    ffffffff ffffffff
0x00040900:    ffffffff ffffffff    ffffffff ffffffff
0x00040910:    ffffffff ffffffff    ffffffff ffffffff
0x00040920:    ffffffff ffffffff    ffffffff ffffffff
0x00040930:    ffffffff ffffffff    ffffffff ffffffff
0x00040940:    ffffffff ffffffff    ffffffff ffffffff
0x00040950:    ffffffff ffffffff    ffffffff ffffffff
0x00040960:    ffffffff ffffffff    ffffffff ffffffff
0x00040970:    ffffffff ffffffff    ffffffff ffffffff
0x00040980:    ffffffff ffffffff    ffffffff ffffffff
0x00040990:    ffffffff ffffffff    ffffffff ffffffff
0x000409a0:    ffffffff ffffffff    ffffffff ffffffff
0x000409b0:    ffffffff ffffffff    ffffffff ffffffff
0x000409c0:    ffffffff ffffffff    ffffffff ffffffff
0x000409d0:    ffffffff ffffffff    ffffffff ffffffff
0x000409e0:    ffffffff ffffffff    ffffffff ffffffff
0x000409f0:    ffffffff ffffffff    ffffffff ffffffff
0x00040a00:    ffffffff ffffffff    ffffffff ffffffff
0x00040a10:    ffffffff ffffffff    ffffffff ffffffff
0x00040a20:    ffffffff ffffffff    ffffffff ffffffff
0x00040a30:    ffffffff ffffffff    ffffffff ffffffff
0x00040a40:    ffffffff ffffffff    ffffffff ffffffff
0x00040a50:    ffffffff ffffffff    ffffffff ffffffff
0x00040a60:    ffffffff ffffffff    ffffffff ffffffff
0x00040a70:    ffffffff ffffffff    ffffffff ffffffff
0x00040a80:    ffffffff ffffffff    ffffffff ffffffff
0x00040a90:    ffffffff ffffffff    ffffffff ffffffff
0x00040aa0:    ffffffff ffffffff    ffffffff ffffffff
0x00040ab0:    ffffffff ffffffff    ffffffff ffffffff
0x00040ac0:    ffffffff ffffffff    ffffffff ffffffff
0x00040ad0:    ffffffff ffffffff    ffffffff ffffffff
0x00040ae0:    ffffffff ffffffff    ffffffff ffffffff
0x00040af0:    ffffffff ffffffff    ffffffff ffffffff
0x00040b00:    ffffffff ffffffff    ffffffff ffffffff
0x00040b10:    ffffffff ffffffff    ffffffff ffffffff
0x00040b20:    ffffffff ffffffff    ffffffff ffffffff
0x00040b30:    ffffffff ffffffff    ffffffff ffffffff
0x00040b40:    ffffffff ffffffff    ffffffff ffffffff
0x00040b50:    ffffffff ffffffff    ffffffff ffffffff
0x00040b60:    ffffffff ffffffff    ffffffff ffffffff
0x00040b70:    ffffffff ffffffff    ffffffff ffffffff
0x00040b80:    ffffffff ffffffff    ffffffff ffffffff
0x00040b90:    ffffffff ffffffff    ffffffff ffffffff
0x00040ba0:    ffffffff ffffffff    ffffffff ffffffff
0x00040bb0:    ffffffff ffffffff    ffffffff ffffffff
0x00040bc0:    ffffffff ffffffff    ffffffff ffffffff
0x00040bd0:    ffffffff ffffffff    ffffffff ffffffff
0x00040be0:    ffffffff ffffffff    ffffffff ffffffff
0x00040bf0:    ffffffff ffffffff    ffffffff ffffffff
0x00040c00:    ffffffff ffffffff    ffffffff ffffffff
0x00040c10:    ffffffff ffffffff    ffffffff ffffffff
0x00040c20:    ffffffff ffffffff    ffffffff ffffffff
0x00040c30:    ffffffff ffffffff    ffffffff ffffffff
0x00040c40:    ffffffff ffffffff    ffffffff ffffffff
0x00040c50:    ffffffff ffffffff    ffffffff ffffffff
0x00040c60:    ffffffff ffffffff    ffffffff ffffffff
0x00040c70:    ffffffff ffffffff    ffffffff ffffffff
0x00040c80:    ffffffff ffffffff    ffffffff ffffffff
0x00040c90:    ffffffff ffffffff    ffffffff ffffffff
0x00040ca0:    ffffffff ffffffff    ffffffff ffffffff
0x00040cb0:    ffffffff ffffffff    ffffffff ffffffff
0x00040cc0:    ffffffff ffffffff    ffffffff ffffffff
0x00040cd0:    ffffffff ffffffff    ffffffff ffffffff
0x00040ce0:    ffffffff ffffffff    ffffffff ffffffff
0x00040cf0:    ffffffff ffffffff    ffffffff ffffffff
0x00040d00:    ffffffff ffffffff    ffffffff ffffffff
0x00040d10:    ffffffff ffffffff    ffffffff ffffffff
0x00040d20:    ffffffff ffffffff    ffffffff ffffffff
0x00040d30:    ffffffff ffffffff    ffffffff ffffffff
0x00040d40:    ffffffff ffffffff    ffffffff ffffffff
0x00040d50:    ffffffff ffffffff    ffffffff ffffffff
0x00040d60:    ffffffff ffffffff    ffffffff ffffffff
0x00040d70:    ffffffff ffffffff    ffffffff ffffffff
0x00040d80:    ffffffff ffffffff    ffffffff ffffffff
0x00040d90:    ffffffff ffffffff    ffffffff ffffffff
0x00040da0:    ffffffff ffffffff    ffffffff ffffffff
0x00040db0:    ffffffff ffffffff    ffffffff ffffffff
0x00040dc0:    ffffffff ffffffff    ffffffff ffffffff
0x00040dd0:    ffffffff ffffffff    ffffffff ffffffff
0x00040de0:    ffffffff ffffffff    ffffffff ffffffff
0x00040df0:    ffffffff ffffffff    ffffffff ffffffff
0x00040e00:    ffffffff ffffffff    ffffffff ffffffff
0x00040e10:    ffffffff ffffffff    ffffffff ffffffff
0x00040e20:    ffffffff ffffffff    ffffffff ffffffff
0x00040e30:    ffffffff ffffffff    ffffffff ffffffff
0x00040e40:    ffffffff ffffffff    ffffffff ffffffff
0x00040e50:    ffffffff ffffffff    ffffffff ffffffff
0x00040e60:    ffffffff ffffffff    ffffffff ffffffff
0x00040e70:    ffffffff ffffffff    ffffffff ffffffff
0x00040e80:    ffffffff ffffffff    ffffffff ffffffff
0x00040e90:    ffffffff ffffffff    ffffffff ffffffff
0x00040ea0:    ffffffff ffffffff    ffffffff ffffffff
0x00040eb0:    ffffffff ffffffff    ffffffff ffffffff
0x00040ec0:    ffffffff ffffffff    ffffffff ffffffff
0x00040ed0:    ffffffff ffffffff    ffffffff ffffffff
0x00040ee0:    ffffffff ffffffff    ffffffff ffffffff
0x00040ef0:    ffffffff ffffffff    ffffffff ffffffff
0x00040f00:    ffffffff ffffffff    ffffffff ffffffff
0x00040f10:    ffffffff ffffffff    ffffffff ffffffff
0x00040f20:    ffffffff ffffffff    ffffffff ffffffff
0x00040f30:    ffffffff ffffffff    ffffffff ffffffff
0x00040f40:    ffffffff ffffffff    ffffffff ffffffff
0x00040f50:    ffffffff ffffffff    ffffffff ffffffff
0x00040f60:    ffffffff ffffffff    ffffffff ffffffff
0x00040f70:    ffffffff ffffffff    ffffffff ffffffff
0x00040f80:    ffffffff ffffffff    ffffffff ffffffff
0x00040f90:    ffffffff ffffffff    ffffffff ffffffff
0x00040fa0:    ffffffff ffffffff    ffffffff ffffffff
0x00040fb0:    ffffffff ffffffff    ffffffff ffffffff
0x00040fc0:    ffffffff ffffffff    ffffffff ffffffff
0x00040fd0:    ffffffff ffffffff    ffffffff ffffffff
0x00040fe0:    ffffffff ffffffff    ffffffff ffffffff
0x00040ff0:    ffffffff ffffffff    ffffffff ffffffff

Example End

```

