## Description

The applet is a small program that loaded into the internal RAM to extend secure ROM capability. 
By the applet user can implement several kinds of needs. In this example OTP content will be dump
by applet mechanism. 
This example will generate .srec file. Then generate SCP (Secure Communication Protocol) package. 

Before build example please set correct silicon (A1, A2...) version you have in MakeFile as below <br />
PROJ_CFLAGS+=-DMAX32662_A1

1. Build the example with scpa configuration ("make scpa")      <br />
2. The SCP package will be generated under scp_packets folder   <br />

## Required Connections

-   Connect a USB cables between the PC and your EvKit connector.
-   Open a console (Powershell or bash) then send scp packages to device.

## Expected Output
    `
    send_scp -c <YOUR_PARTNUMBER> -i UART -s <COMPORT>  scp_packets\packet.list"
    `
[====================================================                    ]  73%


File Name:None                          <br />
Please wait, reading data...            <br />
                                        <br />
****  MAXIM AREA OTP DUMP  ****         <br />
0x10800000: 00 80 02 80 D5 E6 F7 80     <br />
0x10800008: 00 00 80 00 81 D5 66 FB     <br />
0x10800010: 00 00 57 00 00 00 00 80     <br />
0x10800018: A5 A5 5A 5A A5 A5 5A 5A     <br />
0x10800020: A5 A5 5A 5A A5 A5 5A 5A     <br />
0x10800028: A5 A5 5A 5A A5 A5 5A 5A     <br />
0x10800030: FF FF FF FF FF FF FF FF     <br />
0x10800038: FF FF FF FF FF FF FF FF     <br />
0x10800040: 46 12 00 00 05 53 45 59     <br />
0x10800048: 00 00 00 00 00 00 00 80     <br />
0x10800050: 46 12 00 00 05 53 45 59     <br />
0x10800058: 00 00 00 00 00 00 00 80     <br />
0x10800060: A3 5C 7F 3A 20 4F E3 A1     <br />
0x10800068: FF FF FF FF FF FF FF FF     <br />
0x10800070: 68 82 D2 52 2D 2D 00 00     <br />
0x10800078: 00 00 00 00 00 00 00 00     <br />
0x10800080: 68 82 D2 52 2D 2D 00 00     <br />
0x10800088: 00 00 00 00 00 00 00 00     <br />
0x10800090: 68 82 D2 52 2D 2D 00 00     <br />
0x10800098: 00 00 00 00 00 00 00 00     <br />
...

****  END ****                          <br />
[=======================================================                 ]  77%


File Name:None                          <br />
Please wait, reading data...            <br />

****  USER AREA OTP DUMP  ****          <br />
0x10804000: FF FF FF FF FF FF FF FF     <br />
0x10804008: FF FF FF FF FF FF FF FF     <br />
0x10804010: FF FF FF FF FF FF FF FF     <br />
0x10804018: FF FF FF FF FF FF FF FF     <br />
0x10804020: FF FF FF FF FF FF FF FF     <br />
0x10804028: FF FF FF FF FF FF FF FF     <br />
0x10804030: FF FF FF FF FF FF FF FF     <br />
0x10804038: FF FF FF FF FF FF FF FF     <br />
0x10804040: FF FF FF FF FF FF FF FF     <br />
0x10804048: FF FF FF FF FF FF FF FF     <br />
0x10804050: FF FF FF FF FF FF FF FF     <br />
0x10804058: FF FF FF FF FF FF FF FF     <br />
0x10804060: FF FF FF FF FF FF FF FF     <br />
0x10804068: FF FF FF FF FF FF FF FF     <br />
0x10804070: FF FF FF FF FF FF FF FF     <br />
0x10804078: FF FF FF FF FF FF FF FF     <br />
0x10804080: FF FF FF FF FF FF FF FF     <br />
0x10804088: FF FF FF FF FF FF FF FF     <br />
0x10804090: FF FF FF FF FF FF FF FF     <br />
0x10804098: FF FF FF FF FF FF FF FF     <br />
0x108040A0: FF FF FF FF FF FF FF FF     <br />
0x108040A8: FF FF FF FF FF FF FF FF     <br />
0x108040B0: FF FF FF FF FF FF FF FF     <br />
0x108040B8: FF FF FF FF FF FF FF FF     <br />
0x108040C0: FF FF FF FF FF FF FF FF     <br />
0x108040C8: FF FF FF FF FF FF FF FF     <br />
0x108040D0: FF FF FF FF FF FF FF FF     <br />
0x108040D8: FF FF FF FF FF FF FF FF     <br />
0x108040E0: FF FF FF FF FF FF FF FF     <br />
0x108040E8: FF FF FF FF FF FF FF FF     <br />
0x108040F0: FF FF FF FF FF FF FF FF     <br />
0x108040F8: FF FF FF FF FF FF FF FF     <br />

****  END ****                          <br />
[========================================================================] 100%

Disconnected !                          <br />
SCP session OK                          <br />

