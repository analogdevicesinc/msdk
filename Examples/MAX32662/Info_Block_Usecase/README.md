## Description

The MAX32662 has 4KB information block which is between 0x10800000 to 0x10803FFF. The first page of the block (Infoblock 0) is used to keep device related configuration settings and is inaccessible to the user. The second page of the infoblock (Infoblock 1) is reserved for the user. Infoblock 1 is divided into two sections; the first half (0x10802000-0x10802FFF) is write-only and the second half (0x10803000-0x10803FFF) supports both read and write operations.

This example demonstrates how can you read from and write to the user accesible portion of the information block.

To run write test you need to set WITH\_WRITE\_TEST flag in main.c file

For more information please check the MAX32662 User Guide.

## Required Connections
-   Connect a USB cable between the PC and the CN1 (USB/PWR) connector.
-   Install headers JP7(RX\_EN) and JP8(TX\_EN).
-   Open an terminal application on the PC and connect to the EV kit's console UART at 115200, 8-N-1.

## Expected Output

The Console UART of the device will output these messages:

```
***** Info Memory Read/Write Example *****
***** This example demonstrates how you can read/write Info memory *****


***** USER READABLE AREA *****

0x10803000:    11223344 55667788    99aabbcc ddeeff00
0x10803010:    11223344 55667788    99aabbcc ddeeff00
0x10803020:    11223344 55667788    99aabbcc ddeeff00
0x10803030:    ffffffff ffffffff    ffffffff ffffffff
0x10803040:    ffffffff ffffffff    ffffffff ffffffff
0x10803050:    ffffffff ffffffff    ffffffff ffffffff
0x10803060:    ffffffff ffffffff    ffffffff ffffffff
0x10803070:    ffffffff ffffffff    ffffffff ffffffff
0x10803080:    ffffffff ffffffff    ffffffff ffffffff
0x10803090:    ffffffff ffffffff    ffffffff ffffffff
0x108030a0:    ffffffff ffffffff    ffffffff ffffffff
0x108030b0:    ffffffff ffffffff    ffffffff ffffffff
0x108030c0:    ffffffff ffffffff    ffffffff ffffffff
0x108030d0:    ffffffff ffffffff    ffffffff ffffffff
0x108030e0:    ffffffff ffffffff    ffffffff ffffffff
0x108030f0:    ffffffff ffffffff    ffffffff ffffffff
0x10803100:    ffffffff ffffffff    ffffffff ffffffff
0x10803110:    ffffffff ffffffff    ffffffff ffffffff
0x10803120:    ffffffff ffffffff    ffffffff ffffffff
0x10803130:    ffffffff ffffffff    ffffffff ffffffff
0x10803140:    ffffffff ffffffff    ffffffff ffffffff
0x10803150:    ffffffff ffffffff    ffffffff ffffffff
0x10803160:    ffffffff ffffffff    ffffffff ffffffff
0x10803170:    ffffffff ffffffff    ffffffff ffffffff
0x10803180:    ffffffff ffffffff    ffffffff ffffffff
0x10803190:    ffffffff ffffffff    ffffffff ffffffff
0x108031a0:    ffffffff ffffffff    ffffffff ffffffff
0x108031b0:    ffffffff ffffffff    ffffffff ffffffff
0x108031c0:    ffffffff ffffffff    ffffffff ffffffff
0x108031d0:    ffffffff ffffffff    ffffffff ffffffff
0x108031e0:    ffffffff ffffffff    ffffffff ffffffff
0x108031f0:    ffffffff ffffffff    ffffffff ffffffff
0x10803200:    ffffffff ffffffff    ffffffff ffffffff
0x10803210:    ffffffff ffffffff    ffffffff ffffffff
0x10803220:    ffffffff ffffffff    ffffffff ffffffff
0x10803230:    ffffffff ffffffff    ffffffff ffffffff
0x10803240:    ffffffff ffffffff    ffffffff ffffffff
0x10803250:    ffffffff ffffffff    ffffffff ffffffff
0x10803260:    ffffffff ffffffff    ffffffff ffffffff
0x10803270:    ffffffff ffffffff    ffffffff ffffffff
0x10803280:    ffffffff ffffffff    ffffffff ffffffff
0x10803290:    ffffffff ffffffff    ffffffff ffffffff
0x108032a0:    ffffffff ffffffff    ffffffff ffffffff
0x108032b0:    ffffffff ffffffff    ffffffff ffffffff
0x108032c0:    ffffffff ffffffff    ffffffff ffffffff
0x108032d0:    ffffffff ffffffff    ffffffff ffffffff
0x108032e0:    ffffffff ffffffff    ffffffff ffffffff
0x108032f0:    ffffffff ffffffff    ffffffff ffffffff
0x10803300:    ffffffff ffffffff    ffffffff ffffffff
0x10803310:    ffffffff ffffffff    ffffffff ffffffff
0x10803320:    ffffffff ffffffff    ffffffff ffffffff
0x10803330:    ffffffff ffffffff    ffffffff ffffffff
0x10803340:    ffffffff ffffffff    ffffffff ffffffff
0x10803350:    ffffffff ffffffff    ffffffff ffffffff
0x10803360:    ffffffff ffffffff    ffffffff ffffffff
0x10803370:    ffffffff ffffffff    ffffffff ffffffff
0x10803380:    ffffffff ffffffff    ffffffff ffffffff
0x10803390:    ffffffff ffffffff    ffffffff ffffffff
0x108033a0:    ffffffff ffffffff    ffffffff ffffffff
0x108033b0:    ffffffff ffffffff    ffffffff ffffffff
0x108033c0:    ffffffff ffffffff    ffffffff ffffffff
0x108033d0:    ffffffff ffffffff    ffffffff ffffffff
0x108033e0:    ffffffff ffffffff    ffffffff ffffffff
0x108033f0:    ffffffff ffffffff    ffffffff ffffffff
0x10803400:    ffffffff ffffffff    ffffffff ffffffff
0x10803410:    ffffffff ffffffff    ffffffff ffffffff
0x10803420:    ffffffff ffffffff    ffffffff ffffffff
0x10803430:    ffffffff ffffffff    ffffffff ffffffff
0x10803440:    ffffffff ffffffff    ffffffff ffffffff
0x10803450:    ffffffff ffffffff    ffffffff ffffffff
0x10803460:    ffffffff ffffffff    ffffffff ffffffff
0x10803470:    ffffffff ffffffff    ffffffff ffffffff
0x10803480:    ffffffff ffffffff    ffffffff ffffffff
0x10803490:    ffffffff ffffffff    ffffffff ffffffff
0x108034a0:    ffffffff ffffffff    ffffffff ffffffff
0x108034b0:    ffffffff ffffffff    ffffffff ffffffff
0x108034c0:    ffffffff ffffffff    ffffffff ffffffff
0x108034d0:    ffffffff ffffffff    ffffffff ffffffff
0x108034e0:    ffffffff ffffffff    ffffffff ffffffff
0x108034f0:    ffffffff ffffffff    ffffffff ffffffff
0x10803500:    ffffffff ffffffff    ffffffff ffffffff
0x10803510:    ffffffff ffffffff    ffffffff ffffffff
0x10803520:    ffffffff ffffffff    ffffffff ffffffff
0x10803530:    ffffffff ffffffff    ffffffff ffffffff
0x10803540:    ffffffff ffffffff    ffffffff ffffffff
0x10803550:    ffffffff ffffffff    ffffffff ffffffff
0x10803560:    ffffffff ffffffff    ffffffff ffffffff
0x10803570:    ffffffff ffffffff    ffffffff ffffffff
0x10803580:    ffffffff ffffffff    ffffffff ffffffff
0x10803590:    ffffffff ffffffff    ffffffff ffffffff
0x108035a0:    ffffffff ffffffff    ffffffff ffffffff
0x108035b0:    ffffffff ffffffff    ffffffff ffffffff
0x108035c0:    ffffffff ffffffff    ffffffff ffffffff
0x108035d0:    ffffffff ffffffff    ffffffff ffffffff
0x108035e0:    ffffffff ffffffff    ffffffff ffffffff
0x108035f0:    ffffffff ffffffff    ffffffff ffffffff
0x10803600:    ffffffff ffffffff    ffffffff ffffffff
0x10803610:    ffffffff ffffffff    ffffffff ffffffff
0x10803620:    ffffffff ffffffff    ffffffff ffffffff
0x10803630:    ffffffff ffffffff    ffffffff ffffffff
0x10803640:    ffffffff ffffffff    ffffffff ffffffff
0x10803650:    ffffffff ffffffff    ffffffff ffffffff
0x10803660:    ffffffff ffffffff    ffffffff ffffffff
0x10803670:    ffffffff ffffffff    ffffffff ffffffff
0x10803680:    ffffffff ffffffff    ffffffff ffffffff
0x10803690:    ffffffff ffffffff    ffffffff ffffffff
0x108036a0:    ffffffff ffffffff    ffffffff ffffffff
0x108036b0:    ffffffff ffffffff    ffffffff ffffffff
0x108036c0:    ffffffff ffffffff    ffffffff ffffffff
0x108036d0:    ffffffff ffffffff    ffffffff ffffffff
0x108036e0:    ffffffff ffffffff    ffffffff ffffffff
0x108036f0:    ffffffff ffffffff    ffffffff ffffffff
0x10803700:    ffffffff ffffffff    ffffffff ffffffff
0x10803710:    ffffffff ffffffff    ffffffff ffffffff
0x10803720:    ffffffff ffffffff    ffffffff ffffffff
0x10803730:    ffffffff ffffffff    ffffffff ffffffff
0x10803740:    ffffffff ffffffff    ffffffff ffffffff
0x10803750:    ffffffff ffffffff    ffffffff ffffffff
0x10803760:    ffffffff ffffffff    ffffffff ffffffff
0x10803770:    ffffffff ffffffff    ffffffff ffffffff
0x10803780:    ffffffff ffffffff    ffffffff ffffffff
0x10803790:    ffffffff ffffffff    ffffffff ffffffff
0x108037a0:    ffffffff ffffffff    ffffffff ffffffff
0x108037b0:    ffffffff ffffffff    ffffffff ffffffff
0x108037c0:    ffffffff ffffffff    ffffffff ffffffff
0x108037d0:    ffffffff ffffffff    ffffffff ffffffff
0x108037e0:    ffffffff ffffffff    ffffffff ffffffff
0x108037f0:    ffffffff ffffffff    ffffffff ffffffff
0x10803800:    ffffffff ffffffff    ffffffff ffffffff
0x10803810:    ffffffff ffffffff    ffffffff ffffffff
0x10803820:    ffffffff ffffffff    ffffffff ffffffff
0x10803830:    ffffffff ffffffff    ffffffff ffffffff
0x10803840:    ffffffff ffffffff    ffffffff ffffffff
0x10803850:    ffffffff ffffffff    ffffffff ffffffff
0x10803860:    ffffffff ffffffff    ffffffff ffffffff
0x10803870:    ffffffff ffffffff    ffffffff ffffffff
0x10803880:    ffffffff ffffffff    ffffffff ffffffff
0x10803890:    ffffffff ffffffff    ffffffff ffffffff
0x108038a0:    ffffffff ffffffff    ffffffff ffffffff
0x108038b0:    ffffffff ffffffff    ffffffff ffffffff
0x108038c0:    ffffffff ffffffff    ffffffff ffffffff
0x108038d0:    ffffffff ffffffff    ffffffff ffffffff
0x108038e0:    ffffffff ffffffff    ffffffff ffffffff
0x108038f0:    ffffffff ffffffff    ffffffff ffffffff
0x10803900:    ffffffff ffffffff    ffffffff ffffffff
0x10803910:    ffffffff ffffffff    ffffffff ffffffff
0x10803920:    ffffffff ffffffff    ffffffff ffffffff
0x10803930:    ffffffff ffffffff    ffffffff ffffffff
0x10803940:    ffffffff ffffffff    ffffffff ffffffff
0x10803950:    ffffffff ffffffff    ffffffff ffffffff
0x10803960:    ffffffff ffffffff    ffffffff ffffffff
0x10803970:    ffffffff ffffffff    ffffffff ffffffff
0x10803980:    ffffffff ffffffff    ffffffff ffffffff
0x10803990:    ffffffff ffffffff    ffffffff ffffffff
0x108039a0:    ffffffff ffffffff    ffffffff ffffffff
0x108039b0:    ffffffff ffffffff    ffffffff ffffffff
0x108039c0:    ffffffff ffffffff    ffffffff ffffffff
0x108039d0:    ffffffff ffffffff    ffffffff ffffffff
0x108039e0:    ffffffff ffffffff    ffffffff ffffffff
0x108039f0:    ffffffff ffffffff    ffffffff ffffffff
0x10803a00:    ffffffff ffffffff    ffffffff ffffffff
0x10803a10:    ffffffff ffffffff    ffffffff ffffffff
0x10803a20:    ffffffff ffffffff    ffffffff ffffffff
0x10803a30:    ffffffff ffffffff    ffffffff ffffffff
0x10803a40:    ffffffff ffffffff    ffffffff ffffffff
0x10803a50:    ffffffff ffffffff    ffffffff ffffffff
0x10803a60:    ffffffff ffffffff    ffffffff ffffffff
0x10803a70:    ffffffff ffffffff    ffffffff ffffffff
0x10803a80:    ffffffff ffffffff    ffffffff ffffffff
0x10803a90:    ffffffff ffffffff    ffffffff ffffffff
0x10803aa0:    ffffffff ffffffff    ffffffff ffffffff
0x10803ab0:    ffffffff ffffffff    ffffffff ffffffff
0x10803ac0:    ffffffff ffffffff    ffffffff ffffffff
0x10803ad0:    ffffffff ffffffff    ffffffff ffffffff
0x10803ae0:    ffffffff ffffffff    ffffffff ffffffff
0x10803af0:    ffffffff ffffffff    ffffffff ffffffff
0x10803b00:    ffffffff ffffffff    ffffffff ffffffff
0x10803b10:    ffffffff ffffffff    ffffffff ffffffff
0x10803b20:    ffffffff ffffffff    ffffffff ffffffff
0x10803b30:    ffffffff ffffffff    ffffffff ffffffff
0x10803b40:    ffffffff ffffffff    ffffffff ffffffff
0x10803b50:    ffffffff ffffffff    ffffffff ffffffff
0x10803b60:    ffffffff ffffffff    ffffffff ffffffff
0x10803b70:    ffffffff ffffffff    ffffffff ffffffff
0x10803b80:    ffffffff ffffffff    ffffffff ffffffff
0x10803b90:    ffffffff ffffffff    ffffffff ffffffff
0x10803ba0:    ffffffff ffffffff    ffffffff ffffffff
0x10803bb0:    ffffffff ffffffff    ffffffff ffffffff
0x10803bc0:    ffffffff ffffffff    ffffffff ffffffff
0x10803bd0:    ffffffff ffffffff    ffffffff ffffffff
0x10803be0:    ffffffff ffffffff    ffffffff ffffffff
0x10803bf0:    ffffffff ffffffff    ffffffff ffffffff
0x10803c00:    ffffffff ffffffff    ffffffff ffffffff
0x10803c10:    ffffffff ffffffff    ffffffff ffffffff
0x10803c20:    ffffffff ffffffff    ffffffff ffffffff
0x10803c30:    ffffffff ffffffff    ffffffff ffffffff
0x10803c40:    ffffffff ffffffff    ffffffff ffffffff
0x10803c50:    ffffffff ffffffff    ffffffff ffffffff
0x10803c60:    ffffffff ffffffff    ffffffff ffffffff
0x10803c70:    ffffffff ffffffff    ffffffff ffffffff
0x10803c80:    ffffffff ffffffff    ffffffff ffffffff
0x10803c90:    ffffffff ffffffff    ffffffff ffffffff
0x10803ca0:    ffffffff ffffffff    ffffffff ffffffff
0x10803cb0:    ffffffff ffffffff    ffffffff ffffffff
0x10803cc0:    ffffffff ffffffff    ffffffff ffffffff
0x10803cd0:    ffffffff ffffffff    ffffffff ffffffff
0x10803ce0:    ffffffff ffffffff    ffffffff ffffffff
0x10803cf0:    ffffffff ffffffff    ffffffff ffffffff
0x10803d00:    ffffffff ffffffff    ffffffff ffffffff
0x10803d10:    ffffffff ffffffff    ffffffff ffffffff
0x10803d20:    ffffffff ffffffff    ffffffff ffffffff
0x10803d30:    ffffffff ffffffff    ffffffff ffffffff
0x10803d40:    ffffffff ffffffff    ffffffff ffffffff
0x10803d50:    ffffffff ffffffff    ffffffff ffffffff
0x10803d60:    ffffffff ffffffff    ffffffff ffffffff
0x10803d70:    ffffffff ffffffff    ffffffff ffffffff
0x10803d80:    ffffffff ffffffff    ffffffff ffffffff
0x10803d90:    ffffffff ffffffff    ffffffff ffffffff
0x10803da0:    ffffffff ffffffff    ffffffff ffffffff
0x10803db0:    ffffffff ffffffff    ffffffff ffffffff
0x10803dc0:    ffffffff ffffffff    ffffffff ffffffff
0x10803dd0:    ffffffff ffffffff    ffffffff ffffffff
0x10803de0:    ffffffff ffffffff    ffffffff ffffffff
0x10803df0:    ffffffff ffffffff    ffffffff ffffffff
0x10803e00:    ffffffff ffffffff    ffffffff ffffffff
0x10803e10:    ffffffff ffffffff    ffffffff ffffffff
0x10803e20:    ffffffff ffffffff    ffffffff ffffffff
0x10803e30:    ffffffff ffffffff    ffffffff ffffffff
0x10803e40:    ffffffff ffffffff    ffffffff ffffffff
0x10803e50:    ffffffff ffffffff    ffffffff ffffffff
0x10803e60:    ffffffff ffffffff    ffffffff ffffffff
0x10803e70:    ffffffff ffffffff    ffffffff ffffffff
0x10803e80:    ffffffff ffffffff    ffffffff ffffffff
0x10803e90:    ffffffff ffffffff    ffffffff ffffffff
0x10803ea0:    ffffffff ffffffff    ffffffff ffffffff
0x10803eb0:    ffffffff ffffffff    ffffffff ffffffff
0x10803ec0:    ffffffff ffffffff    ffffffff ffffffff
0x10803ed0:    ffffffff ffffffff    ffffffff ffffffff
0x10803ee0:    ffffffff ffffffff    ffffffff ffffffff
0x10803ef0:    ffffffff ffffffff    ffffffff ffffffff
0x10803f00:    ffffffff ffffffff    ffffffff ffffffff
0x10803f10:    ffffffff ffffffff    ffffffff ffffffff
0x10803f20:    ffffffff ffffffff    ffffffff ffffffff
0x10803f30:    ffffffff ffffffff    ffffffff ffffffff
0x10803f40:    ffffffff ffffffff    ffffffff ffffffff
0x10803f50:    ffffffff ffffffff    ffffffff ffffffff
0x10803f60:    ffffffff ffffffff    ffffffff ffffffff
0x10803f70:    ffffffff ffffffff    ffffffff ffffffff
0x10803f80:    ffffffff ffffffff    ffffffff ffffffff
0x10803f90:    ffffffff ffffffff    ffffffff ffffffff
0x10803fa0:    ffffffff ffffffff    ffffffff ffffffff
0x10803fb0:    ffffffff ffffffff    ffffffff ffffffff
0x10803fc0:    ffffffff ffffffff    ffffffff ffffffff
0x10803fd0:    ffffffff ffffffff    ffffffff ffffffff
0x10803fe0:    ffffffff ffffffff    ffffffff ffffffff
0x10803ff0:    ffff55aa ffff55aa    000055aa ffffffff

Example End
```

