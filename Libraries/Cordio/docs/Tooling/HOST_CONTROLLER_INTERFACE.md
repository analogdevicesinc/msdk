# Intro
The Host Controller Interface (HCI) is the common point where devices are split. Typically this interface is over an asynchronous protocol such as UART. Some devices will define proprietary interfaces between the application and host layers. Multi-core SOCs can also use the HCI or proprietary interfaces to split the stack between multiple CPUs.

# Packets
All contents in the packets are formatted little endian unless stated otherwise

## Packet Types

| Packet     | Packet Type |
| ---------- | ----------- |
| Command    | 0x01        |
| Async Data | 0x02        |
| Sync Data  | 0x03        |
| Event      | 0x04        |

### Command Packet

The HCI command packet typically consists of an HCI command header followed by command parameters. The structure of the HCI command packet is defined in the Bluetooth specification.

Here is a general overview of the HCI command packet structure:

| Type (1 Byte) | OpCode (2 Bytes) | Parameters (N Bytes) |
| ------------- | ---------------- | -------------------- |
| 0x01          | 0xXXXX           | ...                  |

Opcodes are mix of Opcode Group Field (OGF, 6 Bits) and the Opcode Command Field (OCF, 10 Bits)

```
Opcode = (OGF << 10)  | OCF
```

Below is an example of the BLE standard command for reset

RESET

| OGF | OCF | Paramters  |
| --- | --- | ---------- |
| 0x3 | 0x3 | Length = 0 |

```
Type = 0x1
Opcode = (0x3 << 10) | 0x3 = 0x0C03
Parmeters = 0

Command = Type | Opcode | Parameters = {0x1, 0x03, 0x0C, 0x00}
```

**Note the little endian format of the opcode**

### Async Data Packet

The asynchronous data packet is comprised of the connection handle, fragmentation bits, the number of data bytes, and the data bytes themselves.

| Handle (12 Bits) | PB Flag (2 Bits) | BC Flag (2 Bits) | Total Length (2 Bytes) | Data (Total Length) |
| ---------------- | ---------------- | ---------------- | ---------------------- | ------------------- |
| 0xXXX            | 0bXX             | 0bXX             | 0xXXXX                 | ...                 |

### Sync Data Packet

This synchronous data packet is not used in BLE.

### Event Data Packet

The structure of an HCI asynchronous event packet typically consists of an HCI event header followed by event parameters. Here's a general overview:

| Type (1 Byte) | Event Code (2 Bytes) | Event Params (N Bytes) |
| ------------- | -------------------- | ---------------------- |
| 0x04          | 0xXX                 | ...                    |



## Vendor Specific Commands

**OGF** : 0x3F

**MAX\_NUMBER\_CONNECTIONS**: set at application layer by user.

NOTE: All data parameters and return values are returned little endian formatted unless stated otherwise.

### Write Memory

Write N bytes to a specified 32-bit address.

Packet type: Command Packet

| **OGF** | **OCF** | ** Length (bytes) ** | * *Parameters**  | **Return** |
| ------- | ------- | ------------------- | ----------------- | ---------- |
| 0x3F    | 0x300   |  5 + N              | {Length, Address} | Status     |

_where N is the number of bytes to write_

#### Parameters (Write Memory)

- _Length_ : 

    | **Description**                      | **Value**         |
    | ------------------------------------ | ----------------- |
    | Number of bytes to write to address | `0 – 0xFF`        |

- _Address_ : Address to read data from 

    | **Description**                      | **Value**         |
    | ------------------------------------ | ----------------- |
    | Address to write data to             | `0 - 0xFFFFFFFF`  |

#### Return (Write Memory)

Returns a status byte.



### Read Memory

Read memory from a specified 32-bit address.

- Packet type: [Command Packet](#command-packet)

| **OGF** | **OCF** |  **Param. Length (bytes)**  | **Parameters**  | **Return** |
| ------- | ------- | --------------------------- | ----------------- | ---------- |
| 0x3F    | 0x301   |  5                          | {Length, Address} | Data (N)   |

#### Parameters (Read Memory)

- _Length_ : 

    | **Description**                      | **Value**         |
    | ------------------------------------ | ----------------- |
    | Number of bytes to read from address | `0 – 0xFF`        |

- _Address_ : Address to read data from 

    | **Description**                      | **Value**         |
    | ------------------------------------ | ----------------- |
    | Address to read data from            | `0 - 0xFFFFFFFF`  |

#### Return (Read Memory)

- _Data_ (N) : 

    | **Description**                           | **Value**        |
    | ----------------------------------------- | ---------------- |
    | _N_ bytes read from the specified address | {`0 - 0xFF`,...} |



### Reset Connection Stats

Clear all connection statistics.

- Packet type: [Command Packet](#command-packet)

| **OGF** | **OCF** | ** Param. Length (bytes) ** | **Parameters** | 
| ------- | ------- | ------------------------- | --------------- | 
| 0x3F    | 0x302   |  0                        | N/A             | 

#### Return (Reset Connection Stats)

Returns a status byte.



### VS TX Test

Start a TX test using a specific number of packets.

- Packet type: [Command Packet](#command-packet)

| **OGF** | **OCF** | **Param. Length (bytes)** | **Parameters**                                      | **Return** |
| ------- | ------- | ------------------------ | --------------------------------------------------- | ---------- |
| 0x3F    | 0x303   | 6                        | {RF Channel, Packet Length, Packet Type, PHY, Num Packets} | Status     |

#### Parameters (VS TX Test)

- _RF Channel (1 Byte):_

    | **Description**           | **Value** |
    | ------------------------- | --------- |
    | RF channel to transmit on | 0 – 39    |

- _Packet Length (1 Byte):_

    | **Description**                  | **Value** |
    | -------------------------------- | --------- |
    | Number of bytes in a single packet | 0 - 255   |

- _Packet Type (1 Byte):_

    | **Description**           | **Value** |
    | ------------------------- | --------- |
    | PRBS9                     | 0x00      |
    | 00001111'b packet payload | 0x01      |
    | 01010101'b packet payload | 0x02      |
    | PRBS15                    | 0x03      |
    | 11111111'b packet payload | 0x04      |
    | 00000000'b packet payload | 0x05      |
    | 11110000'b packet payload | 0x06      |
    | 10101010'b packet payload | 0x07      |

- _PHY (1 Byte):_

    | **Description**   | **Value** |
    | ----------------- | --------- |
    | 1M                | 0x01      |
    | 2M                | 0x02      |
    | Coded Unspecified | 0x03      |
    | Coded S8          | 0x04      |
    | Coded S2          | 0x05      |

- _Num Packets (2 Bytes):_

    | **Description**                                | **Value**  |
    | ---------------------------------------------- | ---------- |
    | Number of packets to send over the course of the test | 0 - 0xFFFF |

#### Return (VS TX Test)

Returns a status byte.



### VS End Test

End the current DTM test and return all test stats.

- Packet type: [Command Packet](#command-packet)

| **OGF** | **OCF** | **Param. Length (bytes)** | **Parameters** | **Return**     |
| ------- | ------- | ------------------------ | -------------- | -------------- |
| 0x3F    | 0x304   | 0                        | N/A            | {TX Data, RX Data Ok, RX Data CRC, RX Data Timeout} |

#### Return (VS End Test)

- _TX Data (2 Bytes)_ : Number of packets transmitted

    | **Description**               | **Value**  |
    | ----------------------------- | ---------- |
    | Number of packets transmitted | 0 - 0xFFFF |

- _RX Data Ok (2 Bytes)_ : Number of packets received ok

    | **Description**               | **Value**  |
    | ----------------------------- | ---------- |
    | Number of packets received ok | 0 – 0xFFFF |

- _RX Data CRC (2 Bytes)_ : Number of packets received with a CRC error

    | **Description**                             | **Value**  |
    | ------------------------------------------- | ---------- |
    | Number of packets received with a CRC error | 0 - 0xFFFF |

- _RX Data Timeout (2 Bytes)_ : Number of timeouts waiting to receive a packet

    | **Description**                                | **Value**  |
    | ---------------------------------------------- | ---------- |
    | Number of timeouts waiting to receive a packet | 0 - 0xFFFF |



### Set Scan Channel Map

Set the channel map to scan on.

- Packet type: [Command Packet](#command-packet)

| **OGF** | **OCF** | **Param. Length (bytes)** | **Parameters** | **Return** |
| ------- | ------- | ------------------------ | -------------- | ---------- |
| 0x3F    | 0x301   | 1                        | Channel Map    | Status     |

#### Parameters (Set Scan Channel Map)

- _Channel Map (1 Byte):_

    | **Description**          | **Value** |
    | ------------------------ | --------- |
    | Channel map used to scan | 0 – 0xFF  |

#### Return (Set Scan Channel Map)

Returns a status byte.



### Set Event Mask

Enables or disables events the controller will flag.

- Packet type: [Command Packet](#command-packet)

| **OGF** | **OCF** | **Param. Length (bytes)** | **Parameters** | **Return** |
| ------- | ------- | ------------------------ | -------------- | ---------- |
| 0x3F    | 0x3E1   | 5                        | {Mask, Enable} | Status     |

#### Parameters (Set Event Mask)

- _Mask (4 Bytes):_

    | **Description**                  | **Value**                |
    | -------------------------------- | ------------------------ |
    | Mask of events to enable/disable | 0x0 – 0xFFFFFFFFFFFFFFFF |

- _Enable (1 Byte):_

    | **Description** | **Value** |
    | --------------- | --------- |
    | Disable         | 0x00      |
    | Enable          | 0x01      |

#### Return (Set Event Mask)

Returns a status byte.




### Enable ACL Sink

Enables or disables asynchronous connection-oriented logical transport.

- Packet type: [Command Packet](#command-packet)

| **OGF** | **OCF** | **Param. Length (bytes)** | **Parameters** | **Return** |
| ------- | ------- | ------------------------ | -------------- | ---------- |
| 0x3F    | 0x3E3   | 1                        | {Enable}       | Status     |

#### Parameters (Enable ACL Sink)

- _Enable (1 Byte):_

    | **Description** | **Value** |
    | --------------- | --------- |
    | Disable         | 0x00      |
    | Enable          | 0x01      |

#### Return (Enable ACL Sink)

Returns a status byte.



### Generate ACL

Generate ACL for a specified connection.

- Packet type: [Command Packet](#command-packet)

| **OGF** | **OCF** | **Param. Length (bytes)** | **Parameters**                      | **Return** |
| ------- | ------- | ------------------------ | ----------------------------------- | ---------- |
| 0x3F    | 0x3E4   | 5                        | {Handle, Packet Length, Num Packets} | Status     |

#### Parameters (Generate ACL)

- _Handle (2 Bytes):_

    | **Description**   | **Value**                     |
    | ----------------- | ----------------------------- |
    | Connection handle | 0x01-MAX\_NUMBER\_CONNECTIONS |

    MAX\_NUMBER\_CONNECTIONS set at the application layer by the user.

- _Packet Length (2 Bytes):_

    | **Description**                   | **Value**            |
    | --------------------------------- | -------------------- |
    | Length of packet of generated ACL | 0x00 – MAX\_ACL\_LEN |

    MAX\_ACL\_LEN set at the application layer by the user.

- _Num Packets (1 Byte):_

    | **Description**                            | **Value**   |
    | ------------------------------------------ | ----------- |
    | Number of packets to send in generated ACL | 0x00 – 0xFF |

#### Return (Generate ACL)

Returns a status byte.




### Enable Autogenerate ACL

Enable or disable Autogenerate ACL.

- Packet type: [Command Packet](#command-packet)

| **OGF** | **OCF** | **Param. Length (bytes)** | **Parameters** | **Return** |
| ------- | ------- | ------------------------ | -------------- | ---------- |
| 0x3F    | 0x3E3   | 1                        | {Enable}       | Status     |

#### Parameters (Enable Autogenerate ACL)

- _Enable (1 Byte):_

    | **Description** | **Value** |
    | --------------- | --------- |
    | Disable         | 0x00      |
    | Enable          | 0x01      |

#### Return (Enable Autogenerate ACL)

- _Status (1 Byte)_ : Status of the Enable Autogenerate ACL command execution.

    | **Return Length (bytes)** | **Value**  |
    | ------------------------- | ---------- |
    | 1                         | Status Code |



### Set TX Test Error Pattern

Set the pattern of errors for the TX test mode.

- Packet type: [Command Packet](#command-packet)

| **OGF** | **OCF** | **Param. Length (bytes)** | **Parameters** | **Return** |
| ------- | ------- | ------------------------ | -------------- | ---------- |
| 0x3F    | 0x3E6   | 4                        | {Error Pattern} | Status     |

#### Parameters (Set TX Test Error Pattern)

- _Error Pattern (1 Byte):_

    | **Description**                 | **Value**        |
    | ------------------------------- | ---------------- |
    | 1s = no error, 0s = CRC Failure | 0x0 - 0xFFFFFFFF |

#### Return (Set TX Test Error Pattern)

Returns a status byte.



### Set Connection Operational Flags

Enable or disable connection operational flags for a given connection.

- Packet type: [Command Packet](#command-packet)

| **OGF** | **OCF** | **Param. Length (bytes)** | **Parameters**             | **Return** |
| ------- | ------- | ------------------------ | -------------------------- | ---------- |
| 0x3F    | 0x3E7   | 7                        | {Handle, Flags, Enable}    | Status     |

#### Parameters (Set Connection Operational Flags)

- _Handle (2 Bytes):_

    | **Description**   | **Value**                     |
    | ----------------- | ----------------------------- |
    | Connection handle | 0x01-MAX\_NUMBER\_CONNECTIONS |

    MAX\_NUMBER\_CONNECTIONS set at the application layer by the user.

- _Flags (4 Bytes):_

    | **Description**            | **Value**        |
    | -------------------------- | ---------------- |
    | Flags to enable or disable | 0x0 – 0xFFFFFFFF |

- _Enable (1 Byte):_

    | **Description** | **Value** |
    | --------------- | --------- |
    | Disable         | 0x00      |
    | Enable          | 0x01      |

#### Return (Set Connection Operational Flags)

Returns a status byte.



### Set P-256 Private Key

Set P-256 private key or clear set private key. The private key will be used for generating key pairs and Diffie-Hellman keys until cleared.

- Packet type: [Command Packet](#command-packet)

| **OGF** | **OCF** | **Param. Length (bytes)** | **Parameters**    | **Return** |
| ------- | ------- | ------------------------ | ----------------- | ---------- |
| 0x3F    | 0x3E8   | 32                       | {Private Key}     | Status     |

#### Parameters (Set P-256 Private Key)

- _Private Key (32 Bytes):_

    | **Description**   | **Value**   |
    | ----------------- | ----------- |
    | Clear Private Key | 0x00…       |
    | Private Key       | 0x1 – 0xFF… |

#### Return (Set P-256 Private Key)

Returns a status byte.



### Get channel map of periodic scan/adv

Get the channel map used for periodic scanning or advertising.

- Packet type: [Command Packet](#command-packet)

| **OGF** | **OCF** | **Param. Length (bytes)** | **Parameters**                | **Return**  |
| ------- | ------- | ------------------------ | ----------------------------- | ----------- |
| 0x3F    | 0x3DE   | 3                        | {Advertising Handle, Advertising} | Channel Map |

#### Parameters (Get channel map of periodic scan/adv)

- _Handle (2 Bytes):_

    | **Description**                    | **Value**                     |
    | ---------------------------------- | ----------------------------- |
    | Periodic Scanner/Advertiser Handle | 0x01-MAX\_NUMBER\_CONNECTIONS |

    MAX\_NUMBER\_CONNECTIONS set at the application layer by the user.

- _Advertising (1 Byte):_

    | **Description** | **Value** |
    | --------------- | --------- |
    | Scanner         | 0x00      |
    | Advertiser      | 0x01      |

#### Return (Get channel map of periodic scan/adv)

- _Channel Map (5 Bytes)_ : Channel map used for periodic scanning or advertising.

    | **Return Length (bytes)** | **Value**      |
    | ------------------------- | -------------- |
    | 5                         | {0x00-0xFF, …} |



### Get ACL Test Report

Get ACL Test Report.

- Packet type: [Command Packet](#command-packet)

| **OGF** | **OCF** | **Param. Length (bytes)** | **Parameters** | **Return**     |
| ------- | ------- | ------------------------ | --------------- | -------------- |
| 0x3F    | 0x3E9   | 0                        | N/A             | {Report Data}  |

#### Return (Get ACL Test Report)

- _RX ACL Packet Count (4 Bytes):_

    | **Description**                | **Value**        |
    | ------------------------------ | ---------------- |
    | Number of ACL packets received | 0x0 – 0xFFFFFFFF |

- _RX ACL Octet Count (4 Bytes):_

    | **Description**               | **Value**        |
    | ----------------------------- | ---------------- |
    | Number of ACL octets received | 0x0 – 0xFFFFFFFF |

- _Generated ACL Packet Count (4 Bytes):_

    | **Description**                 | **Value**        |
    | ------------------------------- | ---------------- |
    | Number of generated ACL packets | 0x0 – 0xFFFFFFFF |

- _Generated ACL Octet Count (4 Bytes):_

    | **Description**                | **Value**        |
    | ------------------------------ | ---------------- |
    | Number of generated ACL octets | 0x0 – 0xFFFFFFFF |



### Set Local Minimum Number of Used Channels

Set local minimum number of used channels.

- Packet type: [Command Packet](#command-packet)

| **OGF** | **OCF** | **Param. Length (bytes)** | **Parameters**                   | **Return** |
| ------- | ------- | ------------------------ | --------------------------------- | ---------- |
| 0x3F    | 0x3EA   | 3                        | {PHYs, Power Threshold, Min Used Channels} | Status     |

#### Parameters (Set Local Minimum Number of Used Channels)

- _PHYs (1 Byte):_

    | **Description** | **Value** |
    | --------------- | --------- |
    | 1M              | 0x1       |
    | 2M              | 0x2       |
    | Coded           | 0x3       |

- _Power Threshold (1 Byte, signed 8-Bit):_

    | **Description**         | **Value** |
    | ----------------------- | --------- |
    | Power Threshold for PHY | +/-127    |

- _Min Used Channels (1 Byte):_

    | **Description**                 | **Value** |
    | ------------------------------- | --------- |
    | Minimum number of used channels | 1 - 37    |

#### Return (Set Local Minimum Number of Used Channels)

Returns a status byte.



### Get Peer Minimum Number of Used Channels

Get peer minimum number of used channels.

- Packet type: [Command Packet](#command-packet)

| **OGF** | **OCF** | **Param. Length (bytes)** | **Parameters** | **Return**     |
| ------- | ------- | ------------------------ | --------------- | -------------- |
| 0x3F    | 0x3EB   | 2                        | {Handle}        | {1M PHY, 2M PHY, Coded PHY} |

#### Parameters (Get Peer Minimum Number of Used Channels)

- _Handle (2 Bytes):_

    | **Description**   | **Value**                     |
    | ----------------- | ----------------------------- |
    | Connection Handle | 0x01-MAX\_NUMBER\_CONNECTIONS |

#### Return (Get Peer Minimum Number of Used Channels)

- _1M PHY (1 Byte):_

    | **Description**      | **Value** |
    | -------------------- | --------- |
    | 1M min used channels | 1 - 37    |

- _2M PHY (1 Byte):_

    | **Description**      | **Value** |
    | -------------------- | --------- |
    | 2M min used channels | 1 - 37    |

- _Coded PHY (1 Byte):_

    | **Description**         | **Value** |
    | ----------------------- | --------- |
    | Coded min used channels | 1 – 37    |



### Set validate public key mode between ALT1 and ALT2

Set mode used to validate public key.

- Packet type: [Command Packet](#command-packet)

| **OGF** | **OCF** | **Param. Length (bytes)** | **Parameters**         | **Return** |
| ------- | ------- | ------------------------ | ---------------------- | ---------- |
| 0x3F    | 0x3EC   | 1                        | {Validate Mode}        | Status     |

#### Parameters (Set validate public key mode between ALT1 and ALT2)

- _Validate Mode (1 Byte):_

    | **Description** | **Value** |
    | --------------- | --------- |
    | ALT2            | 0x0       |
    | ALT1            | 0x1       |

#### Return (Set validate public key mode between ALT1 and ALT2)

Returns a status byte.



### Set BD Address

Set the device address.

- Packet type: [Command Packet](#command-packet)

| **OGF** | **OCF** | **Param. Length (bytes)** | **Parameters**         | **Return** |
| ------- | ------- | ------------------------ | ---------------------- | ---------- |
| 0x3F    | 0x3F0   | 6                        | {BD Address}           | Status     |

#### Parameters (Set BD Address)

- _BD Address (6 Bytes):_

    | **Description** | **Value**      |
    | --------------- | -------------- |
    | Device Address  | {0x00-0xFF, …} |

#### Return (Set BD Address)

Returns a status byte.



### Get Random Address

Get random device address.

- Packet type: [Command Packet](#command-packet)

| **OGF** | **OCF** | **Param. Length (bytes)** | **Parameters** | **Return**     |
| ------- | ------- | ------------------------ | --------------- | -------------- |
| 0x3F    | 0x3F1   | 0                        | N/A             | {BD Address}   |

#### Return (Get Random Address)

- _BD Address (6 Bytes):_

    | **Description** | **Value**      |
    | --------------- | -------------- |
    | Device Address  | {0x00-0xFF, …} |



### Set Local Feature

Set local supported features.

- Packet type: [Command Packet](#command-packet)

| **OGF** | **OCF** | **Param. Length (bytes)** | **Parameters**     | **Return** |
| ------- | ------- | ------------------------ | ------------------ | ---------- |
| 0x3F    | 0x3F2   | 8                        | {Local Features}   | Status     |

#### Parameters (Set Local Feature)

- _Local Features (8 Bytes):_

    | **Description**        | **Value**                 |
    | ---------------------- | ------------------------- |
    | Mask of Local Features | 0x00 – 0xFFFFFFFFFFFFFFFF |

#### Return (Set Local Feature)

- _Status (1 Byte)_ : Status of the Set Local Feature command execution.

    | **Return Length (bytes)** | **Value**  |
    | ------------------------- | ---------- |
    | 1                         | Status Code |



### Set Operational Flags

Enable or Disable Operational Flags.

- Packet type: [Command Packet](#command-packet)

| **OGF** | **OCF** | **Param. Length (bytes)** | **Parameters**     | **Return** |
| ------- | ------- | ------------------------ | ------------------ | ---------- |
| 0x3F    | 0x3F3   | 5                        | {Flags, Enable}    | Status     |

#### Parameters (Set Operational Flags)

- _Flags (4 Bytes):_

    | **Description**            | **Value**        |
    | -------------------------- | ---------------- |
    | Flags to enable or disable | 0x0 – 0xFFFFFFFF |

- _Enable (1 Byte):_

    | **Description** | **Value** |
    | --------------- | --------- |
    | Disable         | 0x00      |
    | Enable          | 0x01      |

#### Return (Set Operational Flags)

Returns a status byte.



### Get PDU Filter Statistics

Get the accumulated PDU filter statistics.

- Packet type: [Command Packet](#command-packet)

| **OGF** | **OCF** | **Param. Length (bytes)** | **Parameters** | **Return** |
| ------- | ------- | ------------------------ | -------------- | ---------- |
| 0x3F    | 0x3F4   | 0                        | N/A            | Statistics |

#### Return (PDU Filter Statistics)

- _Fail PDU Type Filter_ (2 Bytes) :

    | **Description**                        | **Value**    |
    | -------------------------------------- | ------------ |
    | Number of PDUs failing PDU type filter | 0x0 – 0xFFFF |

- _Pass PDU Type Filter_ (2 Bytes) :

    | **Description**                        | **Value**    |
    | -------------------------------------- | ------------ |
    | Number of PDUs passing PDU type filter | 0x0 – 0xFFFF |

- _Fail Whitelist Filter_ (2 Bytes) :

    | **Description**                         | **Value**    |
    | --------------------------------------- | ------------ |
    | Number of PDUs failing whitelist filter | 0x0 – 0xFFFF |

- _Pass Whitelist Filter_ (2 Bytes) :

    | **Description**                          | **Value**    |
    | ---------------------------------------- | ------------ |
    | Number of PDUs passing whitelist filter | 0x0 – 0xFFFF |

- _Fail Peer Address Match_ (2 Bytes) :

    | **Description**                           | **Value**    |
    | ----------------------------------------- | ------------ |
    | Number of PDUs failing peer address match | 0x0 – 0xFFFF |

- _Pass Peer Address Match_ (2 Bytes) :

    | **Description**                           | **Value**    |
    | ----------------------------------------- | ------------ |
    | Number of PDUs passing peer address match | 0x0 – 0xFFFF |

- _Fail Local Address Match_ (2 Bytes) :

    | **Description**                            | **Value**    |
    | ------------------------------------------ | ------------ |
    | Number of PDUs failing local address match | 0x0 – 0xFFFF |

- _Pass Local Address Match_ (2 Bytes) :

    | **Description**                            | **Value**    |
    | ------------------------------------------ | ------------ |
    | Number of PDUs passing local address match | 0x0 – 0xFFFF |

- _Fail Peer RPA Verify_ (2 Bytes) :

    | **Description**                          | **Value**    |
    | ---------------------------------------- | ------------ |
    | Number of peer RPAs failing verification | 0x0 – 0xFFFF |

- _Pass Peer RPA Verify_ (2 Bytes) :

    | **Description**                          | **Value**    |
    | ---------------------------------------- | ------------ |
    | Number of peer RPAs passing verification | 0x0 – 0xFFFF |

- _Fail Local RPA Verify_ (2 Bytes) :

    | **Description**                           | **Value**    |
    | ----------------------------------------- | ------------ |
    | Number of local RPAs failing verification | 0x0 – 0xFFFF |

- _Pass Local RPA Verify_ (2 Bytes) :

    | **Description**                           | **Value**    |
    | ----------------------------------------- | ------------ |
    | Number of local RPAs passing verification | 0x0 – 0xFFFF |

- _Fail Peer Private Address_ (2 Bytes) :

    | **Description**                                         | **Value**    |
    | ------------------------------------------------------- | ------------ |
    | Number of peer addresses failing requirement to be RPAs | 0x0 – 0xFFFF |

- _Fail Local Private Address_ (2 Bytes) :

    | **Description**                                          | **Value**    |
    | -------------------------------------------------------- | ------------ |
    | Number of local addresses failing requirement to be RPAs | 0x0 – 0xFFFF |

- _Fail Peer Address Res Req_ (2 Bytes) :

    | **Description**                                         | **Value**    |
    | ------------------------------------------------------- | ------------ |
    | Number of PDUs failing required peer address resolution | 0x0 – 0xFFFF |

- _Pass Peer Address Res Req_ (2 Bytes) :

    | **Description**                                         | **Value**    |
    | ------------------------------------------------------- | ------------ |
    | Number of PDUs passing optional peer address resolution | 0x0 – 0xFFFF |

- _Pass Local Address Res Opt._ (2 Bytes) :

    | **Description**                                          | **Value**    |
    | -------------------------------------------------------- | ------------ |
    | Number of PDUs passing optional local address resolution | 0x0 – 0xFFFF |

- _Peer Res Address Pend_ (2 Bytes) :

    | **Description**                           | **Value**    |
    | ----------------------------------------- | ------------ |
    | Number of peer address resolutions pended | 0x0 – 0xFFFF |

- _Local Res Address Pend_ (2 Bytes) :

    | **Description**                            | **Value**    |
    | ------------------------------------------ | ------------ |
    | Number of local address resolutions pended | 0x0 – 0xFFFF |



### Set Advertising TX Power

Set the TX power used for advertising.

- Packet type: [Command Packet](#command-packet)

| **OGF** | **OCF** | **Param. Length (bytes)** | **Parameters** | **Return** |
| ------- | ------- | ------------------------ | -------------- | ---------- |
| 0x3F    | 0x3F5   | 1                        | {Power}        | Status     |

#### Parameters (Set Advertising TX Power)

- _Power (1 Byte, Signed 8-Bit):_

    | **Description** | **Value** |
    | --------------- | --------- |
    | Power           | -15 - 6   |

#### Return (Set Advertising TX Power)

Returns a status byte.



### Set Connection TX Power

Set the TX power used for connections.

- Packet type: [Command Packet](#command-packet)

| **OGF** | **OCF** | **Param. Length (bytes)** | **Parameters** | **Return** |
| ------- | ------- | ------------------------ | -------------- | ---------- |
| 0x3F    | 0x3F6   | 1                        | {Power}        | Status     |

#### Parameters (Set Connection TX Power)

- _Power (1 Byte, Signed 8-Bit):_

    | **Description** | **Value** |
    | --------------- | --------- |
    | Power           | -15 - 6   |

#### Return (Set Connection TX Power)

Returns a status byte.



### Set Encryption Mode

Set encryption mode for a given connection.

- Packet type: [Command Packet](#command-packet)

| **OGF** | **OCF** | **Param. Length (bytes)** | **Parameters**           | **Return** |
| ------- | ------- | ------------------------ | ------------------------ | ---------- |
| 0x3F    | 0x3F7   | 4                        | {Enable Auth, Nonce Mode, Handle} | Status     |

#### Parameters (Set Encryption Mode)

- _Enable Auth (1 Byte):_

    | **Description** | **Value** |
    | --------------- | --------- |
    | Disable         | 0x0       |
    | Enable          | 0x1       |

- _Handle (2 Bytes):_

    | **Description**   | **Value**                     |
    | ----------------- | ----------------------------- |
    | Connection Handle | 0x01-MAX\_NUMBER\_CONNECTIONS |

#### Return (Set Encryption Mode)

Returns a status byte.



### Set Channel Map

Set the channel map.

- Packet type: [Command Packet](#command-packet)

| **OGF** | **OCF** | **Param. Length (bytes)** | **Parameters**             | **Return** |
| ------- | ------- | ------------------------ | -------------------------- | ---------- |
| 0x3F    | 0x3F8   | 6                        | {Handle, Channel Map}      | Status     |

#### Parameters (Set Channel Map)

- _Handle (2 Bytes):_

    | **Description**   | **Value**                     |
    | ----------------- | ----------------------------- |
    | Connection Handle | 0x01-MAX\_NUMBER\_CONNECTIONS |

- _Channel Map (4 Bytes):_

    | **Description** | **Value**         |
    | --------------- | ----------------- |
    | Channel Map     | 0x00 – 0xFFFFFFFF |

#### Return (Set Channel Map)

Returns a status byte.



### Set Diagnostic Mode

Enable/Disable PAL System Assert Trap.

- Packet type: [Command Packet](#command-packet)

| **OGF** | **OCF** | **Param. Length (bytes)** | **Parameters** | **Return** |
| ------- | ------- | ------------------------ | -------------- | ---------- |
| 0x3F    | 0x3F9   | 1                        | {Enable}        | Status     |

#### Parameters (Set Diagnostic Mode)

- _Enable (1 Byte):_

    | **Description** | **Value** |
    | --------------- | --------- |
    | Disable         | 0x0       |
    | Enable          | 0x1       |

#### Return (Set Diagnostic Mode)

Returns a status byte.



### Enable Sniffer Packet Forwarding

Enable/Disable sniffer packet forwarding.

- Packet type: [Command Packet](#command-packet)

| **OGF** | **OCF** | **Param. Length (bytes)** | **Parameters** | **Return** |
| ------- | ------- | ------------------------ | -------------- | ---------- |
| 0x3F    | 0x3CD   | 1                        | {Enable}        | Status     |

#### Parameters (Enable Sniffer Packet Forwarding)

- _Enable (1 Byte):_

    | **Description** | **Value** |
    | --------------- | --------- |
    | Disable         | 0x0       |
    | Enable          | 0x1       |

#### Return (Enable Sniffer Packet Forwarding)

Returns a status byte.




### Get Memory Stats

Read memory and system statistics.

- Packet type: [Command Packet](#command-packet)

| **OCF** | **Length** | **Parameters** |
| ------- | ---------- | -------------- |
| 0x3FA   | 0          | N/A            |

#### Return

- _Stack Usage (2 Byte):_

    | **Description**               | **Value**    |
    | ----------------------------- | ------------ |
    | Number of bytes used by stack | 0x0 – 0xFFFF |


- _Sys Assert Count (2 Byte):_

    | **Description**                | **Value**    |
    | ------------------------------ | ------------ |
    | Number of times assertions hit | 0x0 – 0xFFFF |

- _Free Memory (4 Bytes):_

    | **Description**             | **Value**        |
    | --------------------------- | ---------------- |
    | Memory free for stack usage | 0x0 – 0xFFFFFFFF |

- _Used Memory (4 Bytes):_

    | **Description**      | **Value**         |
    | -------------------- | ----------------- |
    | Memory used by stack | 0x00 – 0xFFFFFFFF |

- _Max Connections (2 Bytes):_

    | **Description**                   | **Value**                       |
    | --------------------------------- | ------------------------------- |
    | Number of max connections allowed | 0x00 – MAX\_NUMBER\_CONNECTIONS |

- _Connection Context Size (2 Bytes):_

    | **Description**                             | **Value**     |
    | ------------------------------------------- | ------------- |
    | Number of bytes used for connection context | 0x00 – 0xFFFF |

- _CS Watermark Level (2 Bytes):_

    | **Description**                                     | **Value**     |
    | --------------------------------------------------- | ------------- |
    | Critical section watermark duration in microseconds | 0x00 – 0xFFFF |

- _LL Handler Watermark Level (2 Byte):_

    | **Description**                            | **Value**     |
    | ------------------------------------------ | ------------- |
    | LL handler watermark level in microseconds | 0x00 – 0xFFFF |

- _Sch Handler Watermark Level (2 Byte):_

    | **Description**                                   | **Value**     |
    | ------------------------------------------------- | ------------- |
    | Scheduler handler watermark level in microseconds | 0x00 – 0xFFFF |

- _LHCI Handler Watermark Level (2 Byte):_

    | **Description**                              | **Value**     |
    | -------------------------------------------- | ------------- |
    | LHCI handler watermark level in microseconds | 0x00 – 0xFFFF |

- _Max Adv Sets (2 Bytes):_

    | **Description**                    | **Value**     |
    | ---------------------------------- | ------------- |
    | Maximum number of advertising sets | 0x00 – 0xFFFF |

- _Adv Set Context Size (2 Bytes):_

    | **Description**                          | **Value**     |
    | ---------------------------------------- | ------------- |
    | Size of advertising set context in bytes | 0x00 – 0xFFFF |

- _Ext Scan Max (2 Bytes):_

    | **Description**                     | **Value**    |
    | ----------------------------------- | ------------ |
    | Maximum number of extended scanners | 0x0 – 0xFFFF |

- _Ext Scan Context Size (2 Bytes):_

    | **Description**                                     | **Value**     |
    | --------------------------------------------------- | ------------- |
    | Size of context size for extended scanners in bytes | 0x00 – 0xFFFF |

- _Max Num Extended Init (2 Bytes):_

    | **Description**                        | **Value**     |
    | -------------------------------------- | ------------- |
    | maximum number of extended initiators. | 0x00 – 0xFFFF |

- _Ext Init Context Size (2 Byte):_

    | **Description**                                       | **Value**     |
    | ----------------------------------------------------- | ------------- |
    | Size of context size for extended initiators in bytes | 0x00 – 0xFFFF |

- _Max Periodic Scanners (2 Bytes):_

    | **Description**                     | **Value**   |
    | ----------------------------------- | ----------- |
    | Maximum number of periodic scanners | 0x00-0xFFFF |

- _Periodic Scanners Context Size(2 Bytes):_

    | **Description**                            | **Value**   |
    | ------------------------------------------ | ----------- |
    | Context size of periodic scanners in bytes | 0x00-0xFFFF |

- _Max CIG (2 Bytes):_

    | **Description**       | **Value**   |
    | --------------------- | ----------- |
    | Maximum number of CIG | 0x00-0xFFFF |

- _CIG Context Size (2 Bytes):_

    | **Description**              | **Value**   |
    | ---------------------------- | ----------- |
    | Context size of CIG in bytes | 0x00-0xFFFF |
- _Max CIS (2 Bytes):_

    | **Description**       | **Value**   |
    | --------------------- | ----------- |
    | Maximum number of CIS | 0x00-0xFFFF |

- _CIS Context Size (2 Bytes):_

    | **Description**              | **Value**   |
    | ---------------------------- | ----------- |
    | Context size of CIS in bytes | 0x00-0xFFFF |



### Get Advertising Stats

Get the accumulated advertising statistics.

- Packet type: [Command Packet](#command-packet)

| **OGF** | **OCF** | **Param. Length (bytes)** | **Parameters** | **Return** |
| ------- | ------- | ------------------------ | -------------- | ---------- |
| 0x3F    | 0x3FB   | 0                        | N/A            | Advertising Stats |

#### Return (Advertising Stats)

- _TX ADV (4 Bytes):_

    | **Description**                    | **Value**         |
    | ---------------------------------- | ----------------- |
    | Number of sent advertising packets | 0x00 – 0xFFFFFFFF |

- _RX Req (4 Bytes):_

    | **Description**                                      | **Value**         |
    | ---------------------------------------------------- | ----------------- |
    | Number of successfully received advertising requests | 0x00 – 0xFFFFFFFF |

- _RX Req CRC (4 Bytes):_

    | **Description**                                         | **Value**         |
    | ------------------------------------------------------- | ----------------- |
    | Number of received advertising requests with CRC errors | 0x00 – 0xFFFFFFFF |

- _RX Req Timeout (4 Bytes):_

    | **Description**                                                     | **Value**         |
    | ------------------------------------------------------------------- | ----------------- |
    | Number of timed out received advertising requests (receive timeout) | 0x00 – 0xFFFFFFFF |

- _TX RSP (4 Bytes):_

    | **Description**                 | **Value**         |
    | ------------------------------- | ----------------- |
    | Number of sent response packets | 0x00 – 0xFFFFFFFF |

- _Err ADV (4 Bytes):_

    | **Description**                          | **Value**         |
    | ---------------------------------------- | ----------------- |
    | Number of advertising transaction errors | 0x00 – 0xFFFFFFFF |

- _RX Setup (2 Bytes):_

    | **Description**                           | **Value**     |
    | ----------------------------------------- | ------------- |
    | RX packet setup watermark in microseconds | 0x00 – 0xFFFF |

- _TX Setup (2 Bytes):_

    | **Description**                           | **Value**     |
    | ----------------------------------------- | ------------- |
    | TX packet setup watermark in microseconds | 0x00 – 0xFFFF |

- _RX ISR (2 Bytes):_

    | **Description**                             | **Value**     |
    | ------------------------------------------- | ------------- |
    | RX ISR processing watermark in microseconds | 0x00 – 0xFFFF |

- _TX ISR (2 Bytes):_

    | **Description**                             | **Value**     |
    | ------------------------------------------- | ------------- |
    | TX ISR processing watermark in microseconds | 0x00 – 0xFFFF |



### Get Scan Stats

Get the statistics captured during scanning.

- Packet type: [Command Packet](#command-packet)

| **OGF** | **OCF** | **Param. Length (bytes)** | **Parameters** | **Return** |
| ------- | ------- | ------------------------ | -------------- | ---------- |
| 0x3F    | 0x3FC   | 0                        | N/A            | Scan Stats  |

#### Return (Scan Stats)

- _RX ADV (4 Bytes):_

    | **Description**                                     | **Value**         |
    | --------------------------------------------------- | ----------------- |
    | Number of successfully received advertising packets | 0x00 – 0xFFFFFFFF |

- _RX ADV CRC (4 Bytes):_

    | **Description**                                        | **Value**         |
    | ------------------------------------------------------ | ----------------- |
    | Number of received advertising packets with CRC errors | 0x00 – 0xFFFFFFFF |

- _RX ADV Timeout (4 Bytes):_

    | **Description**                                           | **Value**         |
    | --------------------------------------------------------- | ----------------- |
    | Number of timed out advertising packets (receive timeout) | 0x00 – 0xFFFFFFFF |

- _TX Req (4 Bytes):_

    | **Description**                     | **Value**         |
    | ----------------------------------- | ----------------- |
    | Number of sent advertising requests | 0x00 – 0xFFFFFFFF |

- _RX RSP (4 Bytes):_

    | **Description**                                              | **Value**         |
    | ------------------------------------------------------------ | ----------------- |
    | Number of successfully received advertising response packets | 0x00 – 0xFFFFFFFF |

- _RX RSP CRC (4 Bytes):_

    | **Description**                                                 | **Value**         |
    | --------------------------------------------------------------- | ----------------- |
    | Number of received advertising response packets with CRC errors | 0x00 – 0xFFFFFFFF |

- _RX RSP Timeout (4 Bytes):_

    | **Description**                                                    | **Value**         |
    | ------------------------------------------------------------------ | ----------------- |
    | Number of timed out advertising response packets (receive timeout) | 0x00 – 0xFFFFFFFF |

- _Err Scan (4 Bytes):_

    | **Description**                   | **Value**         |
    | --------------------------------- | ----------------- |
    | Number of scan transaction errors | 0x00 – 0xFFFFFFFF |

- _RX Setup (2 Bytes):_

    | **Description**                           | **Value**         |
    | ----------------------------------------- | ----------------- |
    | RX packet setup watermark in microseconds | 0x00 – 0xFFFFFFFF |

- _TX Setup (2 Bytes):_

    | **Description**                           | **Value**         |
    | ----------------------------------------- | ----------------- |
    | TX packet setup watermark in microseconds | 0x00 – 0xFFFFFFFF |

- _RX ISR (2 Bytes):_

    | **Description**                             | **Value**         |
    | ------------------------------------------- | ----------------- |
    | RX ISR processing watermark in microseconds | 0x00 – 0xFFFFFFFF |

- _TX ISR (2 Bytes):_

    | **Description**                             | **Value**         |
    | ------------------------------------------- | ----------------- |
    | TX ISR processing watermark in microseconds | 0x00 – 0xFFFFFFFF |



### Get Connection Stats

Get the statistics captured during a connection.

- Packet type: [Command Packet](#command-packet)

| **OGF** | **OCF** | **Param. Length (bytes)** | **Parameters** | **Return**     |
| ------- | ------- | ------------------------ | --------------- | -------------- |
| 0x3F    | 0x3FD   | 0                        | N/A             | Connection Stats |

#### Return (Connection Stats)

- _RX Data (4 Bytes):_

    | **Description**                              | **Value**         |
    | -------------------------------------------- | ----------------- |
    | Number of successfully received data packets | 0x00 - 0xFFFFFFFF |

- _RX Data CRC (4 Bytes):_

    | **Description**                                 | **Value**         |
    | ----------------------------------------------- | ----------------- |
    | Number of received data packets with CRC errors | 0x00 - 0xFFFFFFFF |

- _RX Data Timeout (4 Bytes):_

    | **Description**                                    | **Value**         |
    | -------------------------------------------------- | ----------------- |
    | Number of timed out data packets (receive timeout) | 0x00 - 0xFFFFFFFF |

- _TX Data (4 Bytes):_

    | **Description**             | **Value**         |
    | --------------------------- | ----------------- |
    | Number of sent data packets | 0x00 - 0xFFFFFFFF |

- _Err Data (4 Bytes):_

    | **Description**                   | **Value**         |
    | --------------------------------- | ----------------- |
    | Number of data transaction errors | 0x00 - 0xFFFFFFFF |

- _RX Setup (2 Bytes):_

    | **Description**                           | **Value**     |
    | ----------------------------------------- | ------------- |
    | RX packet setup watermark in microseconds | 0x00 - 0xFFFF |

- _TX Setup (2 Bytes):_

    | **Description**                           | **Value**     |
    | ----------------------------------------- | ------------- |
    | TX packet setup watermark in microseconds | 0x00 - 0xFFFF |

- _RX ISR (2 Bytes):_

    | **Description**                             | **Value**     |
    | ------------------------------------------- | ------------- |
    | RX ISR processing watermark in microseconds | 0x00 - 0xFFFF |

- _TX ISR (2 Bytes):_

    | **Description**                             | **Value**     |
    | ------------------------------------------- | ------------- |
    | TX ISR processing watermark in microseconds | 0x00 - 0xFFFF |



### Get Test Stats

Get the statistics captured during test mode.

- Packet type: [Command Packet](#command-packet)

| **OGF** | **OCF** | **Param. Length (bytes)** | **Parameters** | **Return**                        |
| ------- | ------- | ------------------------ | -------------- | --------------------------------- |
| 0x3F    | 0x3FE   | 0                        | N/A            | Test stats in order as documented |

#### Return (Get Test Stats)

- _RX Data (4 Bytes)_

    | **Description**                              | **Value**         |
    | -------------------------------------------- | ----------------- |
    | Number of successfully received data packets | 0x00 - 0xFFFFFFFF |

- _RX Data CRC (4 Bytes)_

    | **Description**                                 | **Value**         |
    | ----------------------------------------------- | ----------------- |
    | Number of received data packets with CRC errors | 0x00 - 0xFFFFFFFF |

- _RX Data Timeout (4 Bytes)_

    | **Description**                                    | **Value**         |
    | -------------------------------------------------- | ----------------- |
    | Number of timed out data packets (receive timeout) | 0x00 - 0xFFFFFFFF |

- _TX Data (4 Bytes)_

    | **Description**             | **Value**         |
    | --------------------------- | ----------------- |
    | Number of sent data packets | 0x00 - 0xFFFFFFFF |

- _Err Data (4 Bytes)_

    | **Description**                   | **Value**         |
    | --------------------------------- | ----------------- |
    | Number of data transaction errors | 0x00 - 0xFFFFFFFF |

- _RX Setup (2 Bytes)_

    | **Description**                           | **Value**     |
    | ----------------------------------------- | ------------- |
    | RX packet setup watermark in microseconds | 0x00 - 0xFFFF |

- _TX Setup (2 Bytes)_

    | **Description**                           | **Value**     |
    | ----------------------------------------- | ------------- |
    | TX packet setup watermark in microseconds | 0x00 - 0xFFFF |

- _RX ISR (2 Bytes)_

    | **Description**                             | **Value**     |
    | ------------------------------------------- | ------------- |
    | RX ISR processing watermark in microseconds | 0x00 - 0xFFFF |

- _TX ISR (2 Bytes)_

    | **Description**                             | **Value**     |
    | ------------------------------------------- | ------------- |
    | TX ISR processing watermark in microseconds | 0x00 - 0xFFFF |



### Get Pool Stats

Get the memory pool statistics captured during runtime.

- Packet type: [Command Packet](#command-packet)

| **OGF** | **OCF** | **Param. Length (bytes)** | **Parameters**  | **Return**                         |
| ------- | ------- | ------------------------ | --------------- | ---------------------------------- |
| 0x3F    | 0x3FF   | 0                        | N/A             | {Num Pool, {Buf Size, Num Buf, Num Alloc, Max Alloc, Max Req Len} * Num Pool} |

#### Return (Get Pool Stats)

- _Num Pool (1 Byte):_

    | **Description**         | **Value**   |
    | ----------------------- | ----------- |
    | Number of pools defined | 0x00 – 0xFF |

- _Pool Stats (variable):_

    | **Description**                   | **Value**     |
    | --------------------------------- | ------------- |
    | _Buf Size (2 Bytes):_             | 0x00 – 0xFFFF |
    | _Num Buf (1 Byte):_               | 0x00 – 0xFF   |
    | _Num Alloc (1 Byte):_             | 0x00 – 0xFF   |
    | _Max Alloc (1 Byte):_             | 0x00 – 0xFF   |
    | _Max Req Len (2 Bytes):_          | 0x00 – 0xFFFF |

### Set Additional AuxPtr Offset

Set auxiliary packet offset delay.

- Packet type: [Command Packet](#command-packet)

| **OGF** | **OCF** | **Param. Length (bytes)** | **Parameters**  | **Return** |
| ------- | ------- | ------------------------ | --------------- | ---------- |
| 0x3F    | 0x3D0   | 5                        | {Delay, Handle} | Status     |

#### Parameters (Set Additional AuxPtr Offset)

- _Delay (4 Bytes):_

    | **Description**       | **Value**        |
    | --------------------- | ---------------- |
    | Disable               | 0x00             |
    | Delay in microseconds | 0x1 – 0xFFFFFFFF |

- _Handle (1 Byte):_

    | **Description**   | **Value**                     |
    | ----------------- | ----------------------------- |
    | Connection handle | 0x01-MAX\_NUMBER\_CONNECTIONS |

#### Return (Set Additional AuxPtr Offset)

Returns a status byte



### Set Extended Advertising Data Fragmentation

Set the extended advertising fragmentation length.

- Packet type: [Command Packet](#command-packet)

| **OGF** | **OCF** | **Param. Length (bytes)** | **Parameters**        | **Return** |
| ------- | ------- | ------------------------ | --------------------- | ---------- |
| 0x3D    | 0x3D1   | 2                        | {Handle, Frag Length} | Status     |

#### Parameters (Set Extended Advertising Data Fragmentation)

- _Handle (1 Byte):_

    | **Description**    | **Value**                     |
    | ------------------ | ----------------------------- |
    | Advertising Handle | 0x01-MAX\_NUMBER\_CONNECTIONS |

- _Frag Length (1 Byte):_

    | **Description**      | **Value** |
    | -------------------- | --------- |
    | Fragmentation Length | 0x00-0xFF |

#### Return (Set Extended Advertising Data Fragmentation)

- _Status (1 Byte)_ : Status of the Set Extended Advertising Data Fragmentation command execution.

    | **Return Length (bytes)** | **Value**  |
    | ------------------------- | ---------- |
    | 1                         | Status Code |



### Set Extended Advertising PHY Options

Set extended advertising PHY options.

- Packet type: [Command Packet](#command-packet)

| **OGF** | **OCF** | **Param. Length (bytes)** | **Parameters**                   | **Return** |
| ------- | ------- | ------------------------ | -------------------------------- | ---------- |
| 0x3D    | 0x3D2   | 3                        | {Handle, Primary Opt., Secondary Opt.} | Status     |

#### Parameters (Set Extended Advertising PHY Options)

- _Handle (1 Byte):_

    | **Description**         | **Value**                     |
    | ----------------------- | ----------------------------- |
    | Advertising Handle      | 0x01-MAX\_NUMBER\_CONNECTIONS |

- _Primary Opt. (1 Byte):_

    | **Description**                          | **Value** |
    | ---------------------------------------- | --------- |
    | Primary advertising channel PHY options. | 0x00-0xFF |

- _Secondary Opt. (1 Byte):_

    | **Description**                            | **Value** |
    | ------------------------------------------ | --------- |
    | Secondary advertising channel PHY options. | 0x00-0xFF |

#### Return (Set Extended Advertising PHY Options)

- _Status (1 Byte)_ : Status of the Set Extended Advertising PHY Options command execution.

    | **Return Length (bytes)** | **Value**  |
    | ------------------------- | ---------- |
    | 1                         | Status Code |



### Set Extended Advertising Default PHY Options

Set the default TX PHY options for extended advertising slave primary and secondary channel.

- Packet type: [Command Packet](#command-packet)

| **OGF** | **OCF** | **Param. Length (bytes)** | **Parameters**  | **Return** |
| ------- | ------- | ------------------------ | --------------- | ---------- |
| 0x3D    | 0x3D3   | 1                        | {PHY Opt.}      | Status     |

#### Parameters (Set Extended Advertising Default PHY Options)

- _PHY Opt. (1 Byte):_

    | **Description** | **Value**   |
    | --------------- | ----------- |
    | PHY Options     | 0x00 – 0xFF |

#### Return (Set Extended Advertising Default PHY Options)

- _Status (1 Byte)_ : Status of the Set Extended Advertising Default PHY Options command execution.

    | **Return Length (bytes)** | **Value**  |
    | ------------------------- | ---------- |
    | 1                         | Status Code |



### Generate ISO Packets

Generate ISO packets.

- Packet type: [Command Packet](#command-packet)

| **OGF** | **OCF** | **Param. Length (bytes)** | **Parameters**                     | **Return** |
| ------- | ------- | ------------------------ | ---------------------------------- | ---------- |
| 0x3D    | 0x3D5   | 5                        | {Handle, Packet Length, Num Packets} | Status     |

#### Parameters (Generate ISO Packets)

- _Handle (2 Byte):_

    | **Description**   | **Value**                     |
    | ----------------- | ----------------------------- |
    | Connection Handle | 0x01-MAX\_NUMBER\_CONNECTIONS |

- _Packet Length (2 Byte):_

    | **Description** | **Value**   |
    | --------------- | ----------- |
    | Packet Length   | 0x00-0xFFFF |

- _Num Packets (1 Byte):_

    | **Description**   | **Value** |
    | ----------------- | --------- |
    | Number of packets | 0x00-0xFF |

#### Return (Generate ISO Packets)

- _Status (1 Byte)_ : Status of the Generate ISO Packets command execution.

    | **Return Length (bytes)** | **Value**  |
    | ------------------------- | ---------- |
    | 1                         | Status Code |



### Get ISO Test Report

Get statistics captured during ISO test.

- Packet type: [Command Packet](#command-packet)

| **OGF** | **OCF** | **Param. Length (bytes)** | **Parameters**                                                                   | **Return**  |
| ------- | ------- | ------------------------ | -------------------------------------------------------------------------------- | ----------- |
| 0x3D    | 0x3D6   | 16                       | {RX ISO Packet Count, RX ISO Octet Count, Generate Packet Count, Generate Octet Count} | Status Code |

#### Return (Get ISO Test Report)

- _RX ISO Packet Count (4 Byte):_

    | **Description**          | **Value**         |
    | ------------------------ | ----------------- |
    | Receive ISO Packet Count | 0x00 – 0xFFFFFFFF |

- _RX ISO Octet Count (4 Byte):_

    | **Description**         | **Value**         |
    | ----------------------- | ----------------- |
    | Receive ISO Octet Count | 0x00 – 0xFFFFFFFF |

- _Generate Packet Count (4 Byte):_

    | **Description**           | **Value**         |
    | ------------------------- | ----------------- |
    | Generate ISO Packet Count | 0x00 – 0xFFFFFFFF |

- _Generate Octet Count (4 Byte):_

    | **Description**          | **Value**         |
    | ------------------------ | ----------------- |
    | Generate ISO Octet Count | 0x00 – 0xFFFFFFFF |



### Enable ISO Packet Sink

Enable or Disable ISO packet sink.

- Packet type: [Command Packet](#command-packet)

| **OGF** | **OCF** | **Param. Length (bytes)** | **Parameters** | **Return** |
| ------- | ------- | ------------------------ | -------------- | ---------- |
| 0x3D    | 0x3D7   | 1                        | {Enable}       | Status     |

#### Parameters (Enable ISO Packet Sink)

- _Enable (1 Byte):_

    | **Description** | **Value** |
    | --------------- | --------- |
    | Disable         | 0x00      |
    | Enable          | 0x01      |

#### Return (Enable ISO Packet Sink)

- _Status (1 Byte)_ : Status of the Enable ISO Packet Sink command execution.

    | **Return Length (bytes)** | **Value**  |
    | ------------------------- | ---------- |
    | 1                         | Status Code |



### Enable Autogenerate ISO Packets

Enable autogenerate ISO packets.

- Packet type: [Command Packet](#command-packet)

| **OGF** | **OCF** | **Param. Length (bytes)** | **Parameters**   | **Return** |
| ------- | ------- | ------------------------ | ---------------- | ---------- |
| 0x3D    | 0x3D8   | 2                        | {Packet Length}  | Status     |

#### Parameters (Enable Autogenerate ISO Packets)

- _Packet Length (2 Bytes):_

    | **Description** | **Value**     |
    | --------------- | ------------- |
    | Disable         | 0x00          |
    | Length          | 0x01 – 0xFFFF |

#### Return (Enable Autogenerate ISO Packets)

- _Status (1 Byte)_ : Status of the Enable Autogenerate ISO Packets command execution.

    | **Return Length (bytes)** | **Value**  |
    | ------------------------- | ---------- |
    | 1                         | Status Code |



### Get ISO Connection Statistics

Get statistics captured during ISO connection.

- Packet type: [Command Packet](#command-packet)

| **OGF** | **OCF** | **Param. Length (bytes)** | **Parameters** | **Return**     |
| ------- | ------- | ------------------------ | -------------- | -------------- |
| 0x3E    | 0x3D9   | 0                        | N/A            | RX, TX, Err... |

#### Return (Get ISO Connection Statistics)

- _RX Data (4 Bytes):_

    | **Description**                              | **Value**         |
    | -------------------------------------------- | ----------------- |
    | Number of successfully received data packets | 0x00 - 0xFFFFFFFF |

- _RX Data CRC (4 Bytes):_

    | **Description**                                 | **Value**         |
    | ----------------------------------------------- | ----------------- |
    | Number of received data packets with CRC errors | 0x00 - 0xFFFFFFFF |

- _RX Data Timeout (4 Bytes):_

    | **Description**                                    | **Value**         |
    | -------------------------------------------------- | ----------------- |
    | Number of timed out data packets (receive timeout) | 0x00 - 0xFFFFFFFF |

- _TX Data (4 Bytes):_

    | **Description**             | **Value**         |
    | --------------------------- | ----------------- |
    | Number of sent data packets | 0x00 - 0xFFFFFFFF |

- _Err Data (4 Bytes):_

    | **Description**                   | **Value**         |
    | --------------------------------- | ----------------- |
    | Number of data transaction errors | 0x00 - 0xFFFFFFFF |

- _RX Setup (2 Bytes):_

    | **Description**                           | **Value**     |
    | ----------------------------------------- | ------------- |
    | RX packet setup watermark in microseconds | 0x00 - 0xFFFF |

- _TX Setup (2 Bytes):_

    | **Description**                           | **Value**     |
    | ----------------------------------------- | ------------- |
    | TX packet setup watermark in microseconds | 0x00 - 0xFFFF |

- _RX ISR (2 Bytes):_

    | **Description**                             | **Value**     |
    | ------------------------------------------- | ------------- |
    | RX ISR processing watermark in microseconds | 0x00 - 0xFFFF |

- _TX ISR (2 Bytes):_

    | **Description**                             | **Value**     |
    | ------------------------------------------- | ------------- |
    | TX ISR processing watermark in microseconds | 0x00 - 0xFFFF |



### Get Auxiliary Advertising Statistics

Get accumulated auxiliary advertising stats.

- Packet type: [Command Packet](#command-packet)

| **OGF** | **OCF** | **Param. Length (bytes)** | **Parameters** | **Return**  |
| ------- | ------- | ------------------------ | -------------- | ----------- |
| 0x3F    | 0x3DA   | 0                        | N/A            | {TX ADV, RX Req, RX Req CRC, RX Req Timeout, TX RSP, TX Chain, Err ADV, RX Setup, TX Setup, RX ISR, TX ISR}  |

#### Return

- _TX ADV (4 Bytes):_

    | **Description**                    | **Value**         |
    | ---------------------------------- | ----------------- |
    | Number of sent advertising packets | 0x00 – 0xFFFFFFFF |

- _RX Req (4 Bytes):_

    | **Description**                                      | **Value**         |
    | ---------------------------------------------------- | ----------------- |
    | Number of successfully received advertising requests | 0x00 – 0xFFFFFFFF |

- _RX Req CRC (4 Bytes):_

    | **Description**                                         | **Value**         |
    | ------------------------------------------------------- | ----------------- |
    | Number of received advertising requests with CRC errors | 0x00 – 0xFFFFFFFF |

- _RX Req Timeout (4 Bytes):_

    | **Description**                                                     | **Value**         |
    | ------------------------------------------------------------------- | ----------------- |
    | Number of timed out received advertising requests (receive timeout) | 0x00 – 0xFFFFFFFF |

- _TX RSP (4 Bytes):_

    | **Description**                 | **Value**         |
    | ------------------------------- | ----------------- |
    | Number of sent response packets | 0x00 – 0xFFFFFFFF |

- _TX Chain (4 Bytes):_

    | **Description**              | **Value**         |
    | ---------------------------- | ----------------- |
    | Number of sent chain packets | 0x00 – 0xFFFFFFFF |

- _Err ADV (4 Bytes):_

    | **Description**                          | **Value**         |
    | ---------------------------------------- | ----------------- |
    | Number of advertising transaction errors | 0x00 – 0xFFFFFFFF |

- _RX Setup (2 Bytes):_

    | **Description**                           | **Value**     |
    | ----------------------------------------- | ------------- |
    | RX packet setup watermark in microseconds | 0x00 – 0xFFFF |

- _TX Setup (2 Bytes):_

    | **Description**                           | **Value**     |
    | ----------------------------------------- | ------------- |
    | TX packet setup watermark in microseconds | 0x00 – 0xFFFF |

- _RX ISR (2 Bytes):_

    | **Description**                             | **Value**     |
    | ------------------------------------------- | ------------- |
    | RX ISR processing watermark in microseconds | 0x00 – 0xFFFF |

- _TX ISR (2 Bytes):_

    | **Description**                             | **Value**     |
    | ------------------------------------------- | ------------- |
    | TX ISR processing watermark in microseconds | 0x00 – 0xFFFF |



### Get Auxiliary Scanning Statistics

Get accumulated auxiliary scanning statistics.

- Packet type: [Command Packet](#command-packet)

| **OGF** | **OCF** | **Param. Length (bytes)** | **Parameters** | **Return**  |
| ------- | ------- | -------------------------- | -------------- | ----------- |
| 0x3F    | 0x3DB   | 0                          | N/A            | Statistics  |

#### Return (Auxiliary Scanning Statistics)

- _RX ADV (4 Bytes):_

    | **Description**                    | **Value**         |
    | ---------------------------------- | ----------------- |
    | Number of sent advertising packets | 0x00 – 0xFFFFFFFF |

- _RX ADV CRC (4 Bytes):_

    | **Description**                                         | **Value**         |
    | ------------------------------------------------------- | ----------------- |
    | Number of received advertising requests with CRC errors | 0x00 – 0xFFFFFFFF |

- _RX ADV Timeout (4 Bytes):_

    | **Description**                                                     | **Value**         |
    | ------------------------------------------------------------------- | ----------------- |
    | Number of timed out received advertising requests (receive timeout) | 0x00 – 0xFFFFFFFF |

- _TX REQ (4 Bytes):_

    | **Description**                     | **Value**         |
    | ----------------------------------- | ----------------- |
    | Number of sent advertising requests | 0x00 – 0xFFFFFFFF |

- _RX RSP (4 Bytes):_

    | **Description**                                              | **Value**         |
    | ------------------------------------------------------------ | ----------------- |
    | Number of successfully received advertising response packets | 0x00 – 0xFFFFFFFF |

- _RX RSP CRC (4 Bytes):_

    | **Description**                                                 | **Value**         |
    | --------------------------------------------------------------- | ----------------- |
    | Number of received advertising response packets with CRC errors | 0x00 – 0xFFFFFFFF |

- _RX RSP Timeout (4 Bytes):_

    | **Description**                                                    | **Value**         |
    | ------------------------------------------------------------------ | ----------------- |
    | Number of timed out advertising response packets (receive timeout) | 0x00 – 0xFFFFFFFF |

- _RX Chain (4 Bytes):_

    | **Description**                               | **Value**         |
    | --------------------------------------------- | ----------------- |
    | Number of successfully received chain packets | 0x00 – 0xFFFFFFFF |

- _RX Chain CRC (4 Bytes):_

    | **Description**                                  | **Value**         |
    | ------------------------------------------------ | ----------------- |
    | Number of received chain packets with CRC errors | 0x00 – 0xFFFFFFFF |

- _RX Chain Timeout (4 Bytes):_

    | **Description**                                     | **Value**         |
    | --------------------------------------------------- | ----------------- |
    | Number of timed out chain packets (receive timeout) | 0x00 – 0xFFFFFFFF |

- _Err Scan (4 Bytes):_

    | **Description**                   | **Value**         |
    | --------------------------------- | ----------------- |
    | Number of scan transaction errors | 0x00 – 0xFFFFFFFF |

- _RX Setup (2 Bytes):_

    | **Description**                           | **Value**     |
    | ----------------------------------------- | ------------- |
    | RX packet setup watermark in microseconds | 0x00 – 0xFFFF |

- _TX Setup (2 Bytes):_

    | **Description**                           | **Value**     |
    | ----------------------------------------- | ------------- |
    | TX packet setup watermark in microseconds | 0x00 – 0xFFFF |

- _RX ISR (2 Bytes):_

    | **Description**                             | **Value**     |
    | ------------------------------------------- | ------------- |
    | RX ISR processing watermark in microseconds | 0x00 – 0xFFFF |

- _TX ISR (2 Bytes):_

    | **Description**                             | **Value**     |
    | ------------------------------------------- | ------------- |
    | TX ISR processing watermark in microseconds | 0x00 – 0xFFFF |




### Get Periodic Scanning Statistics

Get accumulated periodic scanning statistics.

- Packet type: [Command Packet](#command-packet)

| **OGF** | **OCF** | **Param. Length (bytes)** | **Parameters** | **Return** |
| ------- | ------- | -------------------------- | -------------- | ---------- |
| 0x3D    | 0x3DC   | 0                          | N/A            | Statistics |

#### Return (Get Periodic Scanning Statistics)

- _RX ADV (4 Bytes):_

    | **Description**                                     | **Value**         |
    | --------------------------------------------------- | ----------------- |
    | Number of successfully received advertising packets | 0x00 – 0xFFFFFFFF |

- _RX ADV CRC (4 Bytes):_

    | **Description**                                        | **Value**         |
    | ------------------------------------------------------ | ----------------- |
    | Number of received advertising packets with CRC errors | 0x00 – 0xFFFFFFFF |

- _RX ADV Timeout (4 Bytes):_

    | **Description**                                           | **Value**         |
    | --------------------------------------------------------- | ----------------- |
    | Number of timed out advertising packets (receive timeout) | 0x00 – 0xFFFFFFFF |

- _RX Chain (4 Bytes):_

    | **Description**                               | **Value**         |
    | --------------------------------------------- | ----------------- |
    | Number of successfully received chain packets | 0x00 – 0xFFFFFFFF |

- _RX Chain CRC (4 Bytes):_

    | **Description**                                  | **Value**         |
    | ------------------------------------------------ | ----------------- |
    | Number of received chain packets with CRC errors | 0x00 – 0xFFFFFFFF |

- _RX Chain Timeout (4 Bytes):_

    | **Description**                                     | **Value**         |
    | --------------------------------------------------- | ----------------- |
    | Number of timed out chain packets (receive timeout) | 0x00 – 0xFFFFFFFF |

- _Err Scan (4 Bytes):_

    | **Description**                   | **Value**         |
    | --------------------------------- | ----------------- |
    | Number of scan transaction errors | 0x00 – 0xFFFFFFFF |

- _RX Setup (2 Bytes):_

    | **Description**                           | **Value**     |
    | ----------------------------------------- | ------------- |
    | RX packet setup watermark in microseconds | 0x00 – 0xFFFF |

- _TX Setup (2 Bytes):_

    | **Description**                           | **Value**     |
    | ----------------------------------------- | ------------- |
    | TX packet setup watermark in microseconds | 0x00 – 0xFFFF |

- _RX ISR (2 Bytes):_

    | **Description**                             | **Value**     |
    | ------------------------------------------- | ------------- |
    | RX ISR processing watermark in microseconds | 0x00 – 0xFFFF |

- _TX ISR (2 Bytes):_

    | **Description**                             | **Value**     |
    | ------------------------------------------- | ------------- |
    | TX ISR processing watermark in microseconds | 0x00 – 0xFFFF |



### Set Connection PHY TX Power

Set power level during a connection for a given PHY.

- Packet type: [Command Packet](#command-packet)

| **OGF** | **OCF** | **Param. Length (bytes)** | **Parameters**                   | **Return** |
| ------- | ------- | ------------------------ | -------------------------------- | ---------- |
| 0x3F    | 0x3DD   | 4                        | {Handle, Level, PHY}             | Status     |

#### Parameters (Set Connection PHY TX Power)

- _Handle (2 Bytes):_

    | **Description**   | **Value**                     |
    | ----------------- | ----------------------------- |
    | Connection handle | 0x01-MAX\_NUMBER\_CONNECTIONS |

- _Level (1 Byte, Signed 8-Bit):_

    | **Description** | **Value** |
    | --------------- | --------- |
    | Power Level     | -15 - 6   |

- _PHY (1 Byte, Signed 8-Bit):_

    | **Description** | **Value** |
    | --------------- | --------- |
    | 1M              | 0x00      |
    | 2M              | 0x01      |
    | Coded           | 0x02      |

#### Return (Set Connection PHY TX Power)

Returns a status byte


