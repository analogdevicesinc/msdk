# Host Controller Interface (HCI)



## Vendor Specific Commands
OGF = 0x3F
<br>
OPCODE = (OGF << 10) | OCF
 

### Write Memory
- <b>Desciription<b>: Write data to MCU register or memory space at sepcified addresss
- <b>OCF<b>: 0x300
- <b>Length<b>: 5 + N

<b>Parameters<b>


### Read Memory
- <b>Desciription<b>: Write data to MCU register or memory space at sepcified addresss
- <b>OCF<b>: 0x301
- <b>Length<b>: 5
- <b>Parameters<b>: 
  - Length : 1 Bytes
  - Adresss: 4 Bytes
  
### Reset Connection Stats
- <b>Desciription<b>: Write data to MCU register or memory space at sepcified addresss
- <b>OCF<b>
- <b>Length<b>:
- <b>Parameters<b>:
### VS End Test
- <b>Desciription<b>: Write data to MCU register or memory space at sepcified addresss
- <b>OCF<b>:
- <b>Length<b>:
- <b>Parameters<b>:

### VS End Test
- <b>Desciription<b>: Write data to MCU register or memory space at sepcified addresss
- <b>OCF<b>:
- <b>Length<b>:
- <b>Parameters<b>:



|Reset connection stats|Clear all current connection stats|0x302|0|N/A|z|
### VS End Test
- <b>Desciription<b>: Write data to MCU register or memory space at sepcified addresss
- <b>OCF<b>:
- <b>Length<b>:
- <b>Parameters<b>:

|Vendor specific TX test | Normal TX test, but can specify the number of packets to transmit|0x303|6| |x|
### VS End Test
- <b>Desciription<b>: Write data to MCU register or memory space at sepcified addresss
- <b>OCF<b>:
- <b>Length<b>:
- <b>Parameters<b>:

|Vendor specific End Test|0|0x304|None||
### VS End Test
- <b>Desciription<b>: Write data to MCU register or memory space at sepcified addresss
- <b>OCF<b>:
- <b>Length<b>:
- <b>Parameters<b>:

|Enable sniffer packet forwarding| |0x3CD|||
### VS End Test
- <b>Desciription<b>: Write data to MCU register or memory space at sepcified addresss
- <b>OCF<b>:
- <b>Length<b>:
- <b>Parameters<b>:

|Set Additional AuxPtr| |0x3D0||
### VS End Test
- <b>Desciription<b>: Write data to MCU register or memory space at sepcified addresss
- <b>OCF<b>:
- <b>Length<b>:
- <b>Parameters<b>:

| Set extended advertising data fragmentation length | |0x3D1||
### VS End Test
- <b>Desciription<b>: Write data to MCU register or memory space at sepcified addresss
- <b>OCF<b>:
- <b>Length<b>:
- <b>Parameters<b>:

|Set extended advertising PHY options | |0x3D2||
### VS End Test
- <b>Desciription<b>: Write data to MCU register or memory space at sepcified addresss
- <b>OCF<b>:
- <b>Length<b>:
- <b>Parameters<b>:

|Set extended advertising default PHY options | |0x3D3||
### VS End Test
- <b>Desciription<b>: Write data to MCU register or memory space at sepcified addresss
- <b>OCF<b>:
- <b>Length<b>:
- <b>Parameters<b>:

| Set extended scanning default PHY options| |0x3D4||
### VS End Test
- <b>Desciription<b>: Write data to MCU register or memory space at sepcified addresss
- <b>OCF<b>:
- <b>Length<b>:
- <b>Parameters<b>:

| Generate ISO Packets opcode| |0x3D5||
### VS End Test
- <b>Desciription<b>: Write data to MCU register or memory space at sepcified addresss
- <b>OCF<b>:
- <b>Length<b>:
- <b>Parameters<b>:

| Get ISO Test Report opcode| |0x3D6||
### VS End Test
- <b>Desciription<b>: Write data to MCU register or memory space at sepcified addresss
- <b>OCF<b>:
- <b>Length<b>:
- <b>Parameters<b>:

| Enable ISO Packet Sink opcode| |0x3D7||
### VS End Test
- <b>Desciription<b>: Write data to MCU register or memory space at sepcified addresss
- <b>OCF<b>:
- <b>Length<b>:
- <b>Parameters<b>:

| Enable Auto Generate ISO Packets | |0x3D8||
### VS End Test
- <b>Desciription<b>: Write data to MCU register or memory space at sepcified addresss
- <b>OCF<b>:
- <b>Length<b>:
- <b>Parameters<b>:

|  Get ISO Connection Statistics opcode| |0x3D9||
### VS End Test
- <b>Desciription<b>: Write data to MCU register or memory space at sepcified addresss
- <b>OCF<b>:
- <b>Length<b>:
- <b>Parameters<b>:

| Get Auxiliary Advertising Statistics opcode| |0x3DA||
### VS End Test
- <b>Desciription<b>: Write data to MCU register or memory space at sepcified addresss
- <b>OCF<b>:
- <b>Length<b>:
- <b>Parameters<b>:

| Get Auxiliary Scanning Statistics| |0x3DB||
### VS End Test
- <b>Desciription<b>: Write data to MCU register or memory space at sepcified addresss
- <b>OCF<b>:
- <b>Length<b>:
- <b>Parameters<b>:

| Get Periodic Scanning Statistics | |0x3DC||
### VS End Test
- <b>Desciription<b>: Write data to MCU register or memory space at sepcified addresss
- <b>OCF<b>:
- <b>Length<b>:
- <b>Parameters<b>:

| Set Connection Phy Tx Power | |0x3DD||
### VS End Test
- <b>Desciription<b>: Write data to MCU register or memory space at sepcified addresss
- <b>OCF<b>:
- <b>Length<b>:
- <b>Parameters<b>:

|Get channel map of periodic scan/adv| |0x3DE|||
### VS End Test
- <b>Desciription<b>: Write data to MCU register or memory space at sepcified addresss
- <b>OCF<b>:
- <b>Length<b>:
- <b>Parameters<b>:

|Set Scan Channel Map| xc|0x3E0|xx|xx|
### VS End Test
- <b>Desciription<b>: Write data to MCU register or memory space at sepcified addresss
- <b>OCF<b>:
- <b>Length<b>:
- <b>Parameters<b>:

|Set Event Mask| xc|0x3E2|xx|xx|
### VS End Test
- <b>Desciription<b>: Write data to MCU register or memory space at sepcified addresss
- <b>OCF<b>:
- <b>Length<b>:
- <b>Parameters<b>:

|Enable ACL Packet Sink| xc|0x3E3|xx|xx|
### VS End Test
- <b>Desciription<b>: Write data to MCU register or memory space at sepcified addresss
- <b>OCF<b>:
- <b>Length<b>:
- <b>Parameters<b>:

|Generate ACL Packets | xc|0x3E4|xx|xx|
### VS End Test
- <b>Desciription<b>: Write data to MCU register or memory space at sepcified addresss
- <b>OCF<b>:
- <b>Length<b>:
- <b>Parameters<b>:

|Enable Auto Generate ACL Packets | xc|0x3E5||xx|
### VS End Test
- <b>Desciription<b>: Write data to MCU register or memory space at sepcified addresss
- <b>OCF<b>:
- <b>Length<b>:
- <b>Parameters<b>:

|Set Tx Test Error Pattern| |0x3E6|||
### VS End Test
- <b>Desciription<b>: Write data to MCU register or memory space at sepcified addresss
- <b>OCF<b>:
- <b>Length<b>:
- <b>Parameters<b>:

|Set Connection Operational Flags| |0x3E0|||
### VS End Test
- <b>Desciription<b>: Write data to MCU register or memory space at sepcified addresss
- <b>OCF<b>:
- <b>Length<b>:
- <b>Parameters<b>:

|Set P-256 Private Key ||0x3E8|||
### VS End Test
- <b>Desciription<b>: Write data to MCU register or memory space at sepcified addresss
- <b>OCF<b>:
- <b>Length<b>:
- <b>Parameters<b>:

|Get ACL Test Report | |0x3E9|||
### VS End Test
- <b>Desciription<b>: Write data to MCU register or memory space at sepcified addresss
- <b>OCF<b>:
- <b>Length<b>:
- <b>Parameters<b>:

|Set local minimum number of used channels| |0x3EA|||
### VS End Test
- <b>Desciription<b>: Write data to MCU register or memory space at sepcified addresss
- <b>OCF<b>:
- <b>Length<b>:
- <b>Parameters<b>:

|Get peer minimum number of used channels| |0x3EB|||
### VS End Test
- <b>Desciription<b>: Write data to MCU register or memory space at sepcified addresss
- <b>OCF<b>:
- <b>Length<b>:
- <b>Parameters<b>:

|Set validate public key mode between ALT1 and ALT2| |0x3EC|||
### VS End Test
- <b>Desciription<b>: Write data to MCU register or memory space at sepcified addresss
- <b>OCF<b>:
- <b>Length<b>:
- <b>Parameters<b>:

|Set BD address opcode| |0x3F0|||
### VS End Test
- <b>Desciription<b>: Write data to MCU register or memory space at sepcified addresss
- <b>OCF<b>:
- <b>Length<b>:
- <b>Parameters<b>:

|Get Random Address | |0x3F1|||
### VS End Test
- <b>Desciription<b>: Write data to MCU register or memory space at sepcified addresss
- <b>OCF<b>:
- <b>Length<b>:
- <b>Parameters<b>:
|Set Local Feature | |0x3F2|||
### VS End Test
- <b>Desciription<b>: Write data to MCU register or memory space at sepcified addresss
- <b>OCF<b>:
- <b>Length<b>:
- <b>Parameters<b>:

|Set Operational Flags| |0x3F3|||
### VS End Test
- <b>Desciription<b>: Write data to MCU register or memory space at sepcified addresss
- <b>OCF<b>:
- <b>Length<b>:
- <b>Parameters<b>:

|Get PDU Filter Statistics| |0x3F4|||
### VS End Test
- <b>Desciription<b>: Write data to MCU register or memory space at sepcified addresss
- <b>OCF<b>:
- <b>Length<b>:
- <b>Parameters<b>:

|Set Advertising Tx Power| |0x3F5|||
### VS End Test
- <b>Desciription<b>: Write data to MCU register or memory space at sepcified addresss
- <b>OCF<b>:
- <b>Length<b>:
- <b>Parameters<b>:

|Set Connection Tx Power | |0x3F6|||
### VS End Test
- <b>Desciription<b>: Write data to MCU register or memory space at sepcified addresss
- <b>OCF<b>:
- <b>Length<b>:
- <b>Parameters<b>:

|Set Encryption Mode| |0x3F7|||
### VS End Test
- <b>Desciription<b>: Write data to MCU register or memory space at sepcified addresss
- <b>OCF<b>:
- <b>Length<b>:
- <b>Parameters<b>:

|Set Channel Map | |0x3F8|||
### VS End Test
- <b>Desciription<b>: Write data to MCU register or memory space at sepcified addresss
- <b>OCF<b>:
- <b>Length<b>:
- <b>Parameters<b>:

|Set Diagnostic Mode| |0x3F9|||
### VS End Test
- <b>Desciription<b>: Write data to MCU register or memory space at sepcified addresss
- <b>OCF<b>:
- <b>Length<b>:
- <b>Parameters<b>:

|Get Memory Statistics| |0x3FA|||
### VS End Test
- <b>Desciription<b>: Write data to MCU register or memory space at sepcified addresss
- <b>OCF<b>:
- <b>Length<b>:
- <b>Parameters<b>:

|Get Advertising Statistics| |0x3FB|||
### VS End Test
- <b>Desciription<b>: Write data to MCU register or memory space at sepcified addresss
- <b>OCF<b>:
- <b>Length<b>:
- <b>Parameters<b>:

|Get Scan Statistics | |0x3FC|||
### VS End Test
- <b>Desciription<b>: Write data to MCU register or memory space at sepcified addresss
- <b>OCF<b>:
- <b>Length<b>:
- <b>Parameters<b>:

|Get Connection Statistics| |0x3FD|||
### VS End Test
- <b>Desciription<b>: Write data to MCU register or memory space at sepcified addresss
- <b>OCF<b>:
- <b>Length<b>:
- <b>Parameters<b>:

|Get Test Statistics| |0x3FE|||
### VS End Test
- <b>Desciription<b>: Write data to MCU register or memory space at sepcified addresss
- <b>OCF<b>:
- <b>Length<b>:
- <b>Parameters<b>:

|Get Pool Statistics | |0x3FF|||
### VS End Test
- <b>Desciription<b>: Write data to MCU register or memory space at sepcified addresss
- <b>OCF<b>:
- <b>Length<b>:
- <b>Parameters<b>:




