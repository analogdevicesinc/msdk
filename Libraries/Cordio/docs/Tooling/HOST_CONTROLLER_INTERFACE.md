# Intro
The Host Controller Interface (HCI) is the common point where devices are split. Typically this interface is over an asynchronous protocol such as UART. Some devices will define proprietary interfaces between the application and host layers. Multi-core SOCs can also use the HCI or proprietary interfaces to split the stack between multiple CPUs.

# Packets
All contents in the packets are formatted little endian unless stated otherwise
<br>




## Packet Types
| Packet     | Packet Type |
| ---------- | ----------- |
| Command    | 0x01        |
| Async Data | 0x02        |
| Sync Data  | 0x03        |
| Event      | 0x04        |

### Command Type
The HCI command packet typically consists of an HCI command header followed by command parameters. The structure of the HCI command packet is defined in the Bluetooth specification.

Here is a general overview of the HCI command packet structure:

| Type (1 Byte) | OpCode (2 Bytes) | Parameters (N Bytes) |
| ------------- | ---------------- | -------------------- |
| 0x01          | 0xXXXX           | ...                  |

Opcodes are mix of Opcode Group Field (OGF, 6 Bits) and the Opcode Command Field (OCF, 10 Bits)

**Opcode** = (OGF << 10)  | OCF

Below is an example of the BLE standard command for reset

RESET
<br>
| OGF | OCF | Paramters  |
| --- | --- | ---------- |
| 0x3 | 0x3 | Length = 0 |

Type = 0x1
Opcode = (0x3 << 10) | 0x3 = 0x0C03
Parmeters = 0

Command = Type | Opcode | Parameters = {0x1, 0x03, 0x0C, 0x00}

Note the little endian format of the opcode.

### Async Data Type
The asynchronous data packet is comprised of the connection handle, fragmentation bits, the number of data bytes, and the data bytes themselves.
| Handle (12 Bits) | PB Flag (2 Bits) | BC Flag (2 Bits) | Total Length (2 Bytes) | Data (Total Length) |
| ---------------- | ---------------- | ---------------- | ---------------------- | ------------------- |
| 0xXXX            | 0bXX             | 0bXX             | 0xXXXX                 | ...                 |

### Sync Data
This synchronous data packet is not used in BLE.

### Event Data
The structure of an HCI asynchronous event packet typically consists of an HCI event header followed by event parameters. Here's a general overview:
| Type (1 Byte) | Event Code (2 Bytes) | Event Params (N Bytes) |
| ------------- | -------------------- | ---------------------- |
| 0x04          | 0xXX                 | ...                    |



# Vendor Specific Commands

**OGF** : 0x3F
<br>
**MAX\_NUMBER\_CONNECTIONS**: set at application layer by user.

NOTE: All data parameters and return values are returned little endian formatted unless stated otherwise.

### Write Memory

| **OCF** | **Length**                                     | **Parameters**  | **Return** |
| ------- | ---------------------------------------------- | --------------- | ---------- |
| 0x300   | 5 + N, where N is the number of bytes to write | Length, Address | Status     |
|         |
#### Description

Write N bytes to a specified 32-bit address.

#### Parameters

_Length_ (1 Byte):

| **Description**                     | **Value** |
| ----------------------------------- | --------- |
| Number of bytes to write to address | 0 – 0xFF  |
|                                     |
_Address_ (4 Bytes):

| **Description**          | **Value**      |
| ------------------------ | -------------- |
| Address to write data to | 0 - 0xFFFFFFFF |
|                          |

#### Return

Status

### Read Memory

| **OCF** | **Length** | **Parameters**  |
| ------- | ---------- | --------------- |
| 0x301   | 5          | Length, Address |
|         |

#### Description

Read memory from a specified 32-bit address.

#### Parameters

_Length_ (1 Byte):

| **Description**                     | **Value** |
| ----------------------------------- | --------- |
| Number of bytes to write to address | 0 – 0xFF  |

_Address_ (4 Bytes):

| **Description**          | **Value**      |
| ------------------------ | -------------- |
| Address to write data to | 0 - 0xFFFFFFFF |

#### Return

_Data_ (N)

| **Description**        | **Value** |
| ---------------------- | --------- |
| Data read from address | 0 - 0xFF  |

### Reset Connection Stats

| **OCF** | **Length** | **Parameters** |
| ------- | ---------- | -------------- |
| 0x302   | 0          | N/A            |

#### Description

Clear all connection statistics.

#### Return

Status

### VS TX Test

| **OCF** | **Length** | **Parameters**                                      |
| ------- | ---------- | --------------------------------------------------- |
| 0x303   | 6          | RF Channel, Packet Length, Packet Type, Num Packets |
|         |
#### Description

Start a TX test using a specific number of packets.

#### Parameters

_RF Channel_ (1 Byte):

| **Description**           | **Value** |
| ------------------------- | --------- |
| RF channel to transmit on | 0 – 39    |
|                           |

_Packet Length_ (1 Byte):

| **Description**                  | **Value** |
| -------------------------------- | --------- |
| Number of bytes in single packet | 0 - 255   |
|                                  |

_Packet Type (1 Byte)_

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
|                           |

_PHY (1 Byte)_

| **Description**   | **Value** |
| ----------------- | --------- |
| 1M                | 0x01      |
| 2M                | 0x02      |
| Coded Unspecified | 0x03      |
| Coded S8          | 0x04      |
| Coded S2          | 0x05      |
|                   |

_Num Packets (2 Bytes)_

| **Description**                                | **Value**  |
| ---------------------------------------------- | ---------- |
| Number of packets to send over courser of test | 0 - 0xFFFF |

#### Return

Status

### VS End Test

| **OCF** | **Length** | **Parameters** |
| ------- | ---------- | -------------- |
| 0x304   | 0          | N/A            |
|         |
#### Description

End current DTM test and return all test stats.

#### Return

_TX Data (2 Bytes)_

| **Description**               | **Value**  |
| ----------------------------- | ---------- |
| Number of packets transmitted | 0 - 0xFFFF |
|                               |

_RX Data Ok_(2 Bytes)

| **Description**               | **Value**  |
| ----------------------------- | ---------- |
| Number of packets received ok | 0 – 0xFFFF |
|                               |

_RX Data CRC_ (2 Bytes)

| **Description**                             | **Value**  |
| ------------------------------------------- | ---------- |
| Number of packets received with a CRC error | 0 - 0xFFFF |
|                                             |

_RX Data Timeout_ (2 Bytes)

| **Description**                                | **Value**  |
| ---------------------------------------------- | ---------- |
| Number of timeouts waiting to receive a packet | 0 - 0xFFFF |
|                                                |

### Set Scan Channel Map

| **OCF** | **Length** | **Parameters** |
| ------- | ---------- | -------------- |
| 0x301   | 1          | Channel Map    |
|         |

**Description**

Set channel map to scan on

#### Parameters

_Channel Map (1 Byte):_

| **Description**          | **Value** |
| ------------------------ | --------- |
| Channel map used to scan | 0 – 0xFF  |
|                          |
#### Return

Status

### Set Event Mask

| **OCF** | **Length** | **Parameters** |
| ------- | ---------- | -------------- |
| 0x3E1   | 5          | Mask, Enable   |
|         |


#### Description

Enables/Disabled events the controller will flag

**Parameters**

_Mask (4 Bytes):_

| **Description**                  | **Value**                |
| -------------------------------- | ------------------------ |
| Mask of events to enable/disable | 0x0 – 0xFFFFFFFFFFFFFFFF |
|                                  |


_Enable (1 Byte):_

| **Description** | **Value** |
| --------------- | --------- |
| Disable         | 0x00      |
| Enable          | 0x01      |
|                 |

#### Return

Status

### Enable ACL Sink

| **OCF** | **Length** | **Parameters** |
| ------- | ---------- | -------------- |
| 0x3E3   | 1          | Enable         |
|         |


#### Description

Enables/Disables asynchronous connection-oriented logical transport.

#### Parameters

_Enable (1 Byte):_

| **Description** | **Value** |
| --------------- | --------- |
| Disable         | 0x00      |
| Enable          | 0x01      |
|                 |

#### Return

Status

### Generate ACL

| **OCF** | **Length** | **Parameters**                     |
| ------- | ---------- | ---------------------------------- |
| 0x3E4   | 5          | Handle, Packet Length, Num Packets |
|         |


#### Description

Generate ACL for a specified connection.

**Parameters**

_Handle (2 Bytes):_

| **Description**   | **Value**                     |
| ----------------- | ----------------------------- |
| Connection handle | 0x01-MAX\_NUMBER\_CONNECTIONS |
|                   |


MAX\_NUMBER\_CONNECTIONS set at application layer by user.

_Packet Length (2 Bytes):_

| **Description**                   | **Value**            |
| --------------------------------- | -------------------- |
| Length of packet of generated ACL | 0x00 – MAX\_ACL\_LEN |
|                                   |

MAX\_ACL\_LEN set at application layer by user.

Num Packets (1 Bytes):

| **Description**                            | **Value**   |
| ------------------------------------------ | ----------- |
| Number of packets to send in generated ACL | 0x00 – 0xFF |
|                                            |

#### Return

Status

### Enable Autogenerate ACL

| **OCF** | **Length** | **Parameters** |
| ------- | ---------- | -------------- |
| 0x3E3   | 1          | Enable         |
|         |

#### Description

Enable/Disable Autogenerate ACL

#### Parameters

_Enable (1 Byte):_

| **Description** | **Value** |
| --------------- | --------- |
| Disable         | 0x00      |
| Enable          | 0x01      |
|                 |


#### Return

Status

### Set TX Test Error Pattern

| **OCF** | **Length** | **Parameters** |
| ------- | ---------- | -------------- |
| 0x3E6   | 4          | Error Pattern  |
|         |

#### Description

Set pattern of errors for TX test mode.

**Parameters**

_Error Pattern (1 Byte):_

| **Description**                 | **Value**        |
| ------------------------------- | ---------------- |
| 1s = no error, 0s = CRC Failure | 0x0 - 0xFFFFFFFF |
|                                 |

#### Return

Status

### Set Connection Operational Flags

| **OCF** | **Length** | **Parameters**        |
| ------- | ---------- | --------------------- |
| 0x3E7   | 7          | Handle, Flags, Enable |
|         |


#### Description

Enable/Disable connection operational flags for a given connection.

#### Parameters

_Handle (2 Bytes):_

| **Description**   | **Value**                     |
| ----------------- | ----------------------------- |
| Connection handle | 0x01-MAX\_NUMBER\_CONNECTIONS |
|                   |


MAX\_NUMBER\_CONNECTIONS set at application layer by user.

_Flags (4 Bytes)_

| **Description**            | **Value**        |
| -------------------------- | ---------------- |
| Flags to enable or disable | 0x0 – 0xFFFFFFFF |
|                            |


_Enable (1 Byte):_

| **Description** | **Value** |
| --------------- | --------- |
| Disable         | 0x00      |
| Enable          | 0x01      |
|                 |

#### Return

Status

### Set P-256 Private Key

| **OCF** | **Length** | **Parameters** |
| ------- | ---------- | -------------- |
| 0x3E8   | 32         | Private Key    |
|         |


#### Description

Set P-256 private key or clear set private key. The private key will be used for generating key

pairs and Diffie-Hellman keys until cleared.

#### Parameters

_Private Key (32 Bytes):_

| **Description**   | **Value**   |
| ----------------- | ----------- |
| Clear Private Key | 0x00…       |
| Private Key       | 0x1 – 0xFF… |
|                   |


#### Return

Status

### Get channel map of periodic scan/adv

| **OCF** | **Length** | **Parameters**                | **Return**  |
| ------- | ---------- | ----------------------------- | ----------- |
| 0x3DE   | 3          | Advertising Handle, Adverting | Channel Map |
|         |

#### Description

Get the channel map used for periodic scanning or advertising.

#### Parameters

_Handle (2 Bytes):_

| **Description**                    | **Value**                     |
| ---------------------------------- | ----------------------------- |
| Periodic Scanner/Advertiser Handle | 0x01-MAX\_NUMBER\_CONNECTIONS |
|                                    |



_Advertising (1 Byte):_

| **Description** | **Value** |
| --------------- | --------- |
| Scanner         | 0x00      |
| Advertiser      | 0x01      |
|                 |

#### Return

_Channel Map (5 Bytes):_

| **Description** | **Value**      |
| --------------- | -------------- |
| Channel Masks   | {0x00-0xFF, …} |
|                 |

### Get ACL Test Report

| **OCF** | **Length** | **Parameters** |
| ------- | ---------- | -------------- |
| 0x3E9   | 0          | **N/A**        |
|         |

#### Description

Get ACL Test Report

#### Return

_RX ACL Packet Count (4 Bytes):_

| **Description**                | **Value**        |
| ------------------------------ | ---------------- |
| Number of ACL packets received | 0x0 – 0xFFFFFFFF |
|                                |

_RX ACL Octet Count (4 Bytes):_

| **Description**               | **Value**        |
| ----------------------------- | ---------------- |
| Number of ACL octets received | 0x0 – 0xFFFFFFFF |
|                               |


_Generated ACL Packet Count (4 Bytes):_

| **Description**                 | **Value**        |
| ------------------------------- | ---------------- |
| Number of generated ACL packets | 0x0 – 0xFFFFFFFF |
|                                 |


_Generated ACL Octet Count (4 Bytes):_

| **Description**                | **Value**        |
| ------------------------------ | ---------------- |
| Number of generated ACL octets | 0x0 – 0xFFFFFFFF |
|                                |

### Set Local Minimum Number of Used Channels

| **OCF** | **Length** | **Parameters**                           |
| ------- | ---------- | ---------------------------------------- |
| 0x3EA** | 3          | PHYs, Power Threshold, Min Used Channels |
|         |


#### Description

Set local minimum number of used channels.

#### Parameters

_PHYs (1 Byte):_

| **Description** | **Value** |
| --------------- | --------- |
| 1M              | 0x1       |
| 2M              | 0x2       |
| Coded           | 0x3       |
|                 |


_Power Threshold (1 Byte, signed 8-Bit):_

| **Description**         | **Value** |
| ----------------------- | --------- |
| Power Threshold for PHY | +/-127    |
|                         |


Min Used Channels (1 Byte)

| **Description**                 | **Value** |
| ------------------------------- | --------- |
| Minimum number of used channels | 1 - 37    |
|                                 |


#### Return

Status

### Get Peer Minimum Number of Used Channels

| **OCF** | **Length** | **Parameters** | **Return**                |
| ------- | ---------- | -------------- | ------------------------- |
| 0x3EB   | 2          | Handle         | 1M PHY, 2M PHY, Coded PHY |
|         |

#### Description

Get peer minimum number of used channels.

#### Parameters

_Handle (2 Bytes):_

| **Description**   | **Value**                     |
| ----------------- | ----------------------------- |
| Connection Handle | 0x01-MAX\_NUMBER\_CONNECTIONS |
|                   |


#### Return

_1M PHY (1 Byte):_

| **Description**      | **Value** |
| -------------------- | --------- |
| 1M min used channels | 1 - 37    |
|                      |

_2M PHY (1 Byte):_

| **Description**      | **Value** |
| -------------------- | --------- |
| 2M min used channels | 1 - 37    |
|                      |

_Coded PHY (1 Byte):_

| **Description**         | **Value** |
| ----------------------- | --------- |
| Coded min used channels | 1 – 37    |
|                         |

### Set validate public key mode between ALT1 and ALT2

| **OCF**   | **Length** | **Parameters**    |
| --------- | ---------- | ----------------- |
| **0x3EC** | **1**      | **Validate Mode** |
|           |


#### Description

Set mode used to validate public key.

#### Parameters

_Validate Mode (1 Byte):_

| **Description** | **Value** |
| --------------- | --------- |
| ALT2            | 0x0       |
| ALT1            | 0x1       |
|                 |

#### Return

Status

### Set BD Address

| **OCF** | **Length** | **Parameters** |
| ------- | ---------- | -------------- |
| 0x3F0   | 6          | BD Address     |
|         |


#### Description

Set the device address.

#### Parameters

_BD Address (6 Bytes):_

| **Description** | **Value**      |
| --------------- | -------------- |
| Device Address  | {0x00-0xFF, …} |
|                 |


#### Return

Status

### Get Random Address

| **OCF** | **Length** | **Parameters** | **Return** |
| ------- | ---------- | -------------- | ---------- |
| 0x3F1   | 0          | N/A            | BD Address |
|         |
#### Description

Get random device address.

#### Return

_BD Address (6 Bytes):_

| **Description** | **Value**      |
| --------------- | -------------- |
| Device Address  | {0x00-0xFF, …} |
|                 |
### Set Local Feature

| **OCF** | **Length** | **Parameters** |
| ------- | ---------- | -------------- |
| 0x3F2   | 8          | Local Features |
|         |


#### Description

Set local supported features.

#### Parameters

_Local Features (8 Bytes):_

| **Description**        | **Value**                 |
| ---------------------- | ------------------------- |
| Mask of Local Features | 0x00 – 0xFFFFFFFFFFFFFFFF |
|                        |


#### Return

Status

### Set Operational Flags

| **OCF** | **Length** | **Parameters** |
| ------- | ---------- | -------------- |
| 0x3F3   | 5          | Flags, Enable  |
|         |


#### Description

Enable Disable Operational Flags

#### Parameters

_Flags_ (4 Bytes):

| **Description**            | **Value**        |
| -------------------------- | ---------------- |
| Flags to enable or disable | 0x0 – 0xFFFFFFFF |
|                            |

_Enable (1 Byte):_

| **Description** | **Value** |
| --------------- | --------- |
| Disable         | 0x00      |
| Enable          | 0x01      |
|                 |

#### Return

Status

### Get PDU Filter Statistics

| **OCF** | **Length** | **Parameters** |
| ------- | ---------- | -------------- |
| 0x3F4   | 0          | N/A            |
|         |

#### Description

Get the accumulated PDU filter statistics.

#### Return

_Fail PDU Type Filter_ (2 Bytes):

| **Description**                        | **Value**    |
| -------------------------------------- | ------------ |
| Number of PDUs failing PDU type filter | 0x0 – 0xFFFF |
|                                        |

_Pass PDU Type Filter_ (2 Bytes):

| **Description**                        | **Value**    |
| -------------------------------------- | ------------ |
| Number of PDUs passing PDU type filter | 0x0 – 0xFFFF |
|                                        |

_Fail Whitelist Filter_ (2 Bytes):

| **Description**                         | **Value**    |
| --------------------------------------- | ------------ |
| Number of PDUs failing whitelist filter | 0x0 – 0xFFFF |
|                                         |

_Pass Whitelist Filter_ (2 Bytes):

| **Description**                         | **Value**    |
| --------------------------------------- | ------------ |
| Number of PDUs passing whitelist filter | 0x0 – 0xFFFF |
|                                         |

_Fail Peer Address Match_ (2 Bytes):

| **Description**                           | **Value**    |
| ----------------------------------------- | ------------ |
| Number of PDUS failing peer address match | 0x0 – 0xFFFF |
|                                           |

_Pass Peer Address Match_ (2 Bytes):

| **Description**                           | **Value**    |
| ----------------------------------------- | ------------ |
| Number of PDUs passing peer address match | 0x0 – 0xFFFF |
|                                           |

_Fail Local Address Match_ (2 Bytes):

| **Description**                            | **Value**    |
| ------------------------------------------ | ------------ |
| Number of PDUS failing local address match | 0x0 – 0xFFFF |
|                                            |

_Pass local Address Match_ (2 Bytes):

| **Description**                            | **Value**    |
| ------------------------------------------ | ------------ |
| Number of PDUs passing local address match | 0x0 – 0xFFFF |
|                                            |

_Fail Peer RPA Verify_ (2 Bytes):

| **Description**                          | **Value**    |
| ---------------------------------------- | ------------ |
| Number of peer RPAs failing verification | 0x0 – 0xFFFF |
|                                          |

_Pass Peer RPA Verify_ (2 Bytes):

| **Description**                          | **Value**    |
| ---------------------------------------- | ------------ |
| Number of peer RPAs passing verification | 0x0 – 0xFFFF |
|                                          |

_Fail Local RPA Verify_ (2 Bytes):

| **Description**                           | **Value**    |
| ----------------------------------------- | ------------ |
| Number of local RPAs failing verification | 0x0 – 0xFFFF |
|                                           |

_Pass Local RPA Verify_ (2 Bytes):

| **Description**                           | **Value**    |
| ----------------------------------------- | ------------ |
| Number of local RPAs passing verification | 0x0 – 0xFFFF |
|                                           |

_Fail Peer Private Address_ (2 Bytes):

| **Description**                                         | **Value**    |
| ------------------------------------------------------- | ------------ |
| Number of peer addresses failing requirement to be RPAs | 0x0 – 0xFFFF |
|                                                         |

_Fail Local Private Address_ (2 Bytes):

| **Description**                                          | **Value**    |
| -------------------------------------------------------- | ------------ |
| Number of local addresses failing requirement to be RPAs | 0x0 – 0xFFFF |
|                                                          |

_Fail Peer Address Res Req_ (2 Bytes):

| **Description**                                         | **Value**    |
| ------------------------------------------------------- | ------------ |
| Number of PDUs failing required peer address resolution | 0x0 – 0xFFFF |
|                                                         |

_Pass Peer Address Res Req_ (2 Bytes):

| **Description**                                         | **Value**    |
| ------------------------------------------------------- | ------------ |
| Number of PDUs passing optional peer address resolution | 0x0 – 0xFFFF |
|                                                         |

_Pass Local Address Res Opt._ (2 Bytes):
| **Description**                                          | **Value**    |
| -------------------------------------------------------- | ------------ |
| Number of PDUs passing optional local address resolution | 0x0 – 0xFFFF |
|                                                          |

_Peer Res Address Pend_ (2 Bytes):

| **Description**                           | **Value**    |
| ----------------------------------------- | ------------ |
| Number of peer address resolutions pended | 0x0 – 0xFFFF |
|                                           |

_Local Res Address Pend_ (2 Bytes):

| **Description**                            | **Value**    |
| ------------------------------------------ | ------------ |
| Number of local address resolutions pended | 0x0 – 0xFFFF |
|                                            |
### Set Advertising TX Power

| **OCF** | **Length** | **Parameters** | **Return** |
| ------- | ---------- | -------------- | ---------- |
| 0x3F5   | 1          | Power          | Status     |
|         |
#### Description

Set the TX power used for advertising.

#### Parameters

_Power (1 Byte, Signed 8-Bit):_

| **Description** | **Value** |
| --------------- | --------- |
| Power           | -15 - 6   |
|                 |


#### Return

Status

### Set Connection TX Power

| **OCF** | **Length** | **Parameters** | **Return** |
| ------- | ---------- | -------------- | ---------- |
| 0x3F6   | 1          | Power          | Status     |
|         |
#### Description

Set the TX power used for connections.

#### Parameters

_Power (1 Byte, Signed 8-Bit):_

| **Description** | **Value** |
| --------------- | --------- |
| Power           | -15 - 6   |
|                 |


#### Return

Status

### Set Encryption Mode

| **OCF** | **Length** | **Parameters**                  |
| ------- | ---------- | ------------------------------- |
| 0x3F7   | 4          | Enable Auth, Nonce Mode, Handle |
|         |


#### Description

Set encryption mode for a given connection.

#### Parameters

_Enable Auth (1 Byte):_

| **Description** | **Value** |
| --------------- | --------- |
| Disable         | 0x0       |
| Enable          | 0x1       |
|                 |


_Handle (2 Bytes):_

| **Description**   | **Value**                     |
| ----------------- | ----------------------------- |
| Connection Handle | 0x01-MAX\_NUMBER\_CONNECTIONS |
|                   |


#### Return

Status

### Set Channel Map

| **OCF** | **Length** | **Parameters**      |
| ------- | ---------- | ------------------- |
| 0x3F8   | 6          | Handle, Channel Map |
|         |


#### Description

Set the channel map

#### Parameters

_Handle (2 Bytes):_

| **Description**   | **Value**                     |
| ----------------- | ----------------------------- |
| Connection Handle | 0x01-MAX\_NUMBER\_CONNECTIONS |
|                   |


_Channel Map (4 Bytes):_

| **Description** | **Value**         |
| --------------- | ----------------- |
| Channel Map     | 0x00 – 0xFFFFFFFF |
|                 |

#### Return

Status

### Set Diagnostic Mode

| **OCF** | **Length** | **Parameters** |
| ------- | ---------- | -------------- |
| 0x3F9   | 1          | Enable         |
|         |


#### Description

Enable/Disable PAL System Assert Trap

#### Parameters

_Enable (1 Byte):_

| **Description** | **Value** |
| --------------- | --------- |
| Disable         | 0x0       |
| Enable          | 0x1       |
|                 |


#### Return

Status

### Enable Sniffer Packet Forwarding

| **OCF** | **Length** | **Parameters** |
| ------- | ---------- | -------------- |
| 0x3CD   | 1          | Enable         |
|         |


#### Description

Enable/Disable sniffer packet forwarding.

#### Parameters

_Enable (1 Byte):_

| **Description** | **Value** |
| --------------- | --------- |
| Disable         | 0x0       |
| Enable          | 0x1       |
|                 |


#### Return

Status

### Get Memory Stats

| **OCF** | **Length** | **Parameters** |
| ------- | ---------- | -------------- |
| 0x3FA   | 0          | N/A            |
|         |

#### Description

Read memory and system statistics.

#### Return

_Stack Usage (2 Byte):_

| **Description**               | **Value**    |
| ----------------------------- | ------------ |
| Number of bytes used by stack | 0x0 – 0xFFFF |
|                               |


_Sys Assert Count (2 Byte):_

| **Description**                | **Value**    |
| ------------------------------ | ------------ |
| Number of times assertions hit | 0x0 – 0xFFFF |
|                                |

_Free Memory (4 Bytes):_

| **Description**             | **Value**        |
| --------------------------- | ---------------- |
| Memory free for stack usage | 0x0 – 0xFFFFFFFF |
|                             |

_Used Memory (4 Bytes):_

| **Description**      | **Value**         |
| -------------------- | ----------------- |
| Memory used by stack | 0x00 – 0xFFFFFFFF |
|                      |

_Max Connections (2 Bytes):_

| **Description**                   | **Value**                       |
| --------------------------------- | ------------------------------- |
| Number of max connections allowed | 0x00 – MAX\_NUMBER\_CONNECTIONS |
|                                   |

_Connection Context Size (2 Bytes):_

| **Description**                             | **Value**     |
| ------------------------------------------- | ------------- |
| Number of bytes used for connection context | 0x00 – 0xFFFF |
|                                             |

_CS Watermark Level (2 Bytes):_

| **Description**                                     | **Value**     |
| --------------------------------------------------- | ------------- |
| Critical section watermark duration in microseconds | 0x00 – 0xFFFF |
|                                                     |

_LL Handler Watermark Level (2 Byte):_

| **Description**                            | **Value**     |
| ------------------------------------------ | ------------- |
| LL handler watermark level in microseconds | 0x00 – 0xFFFF |
|                                            |

_Sch Handler Watermark Level (2 Byte):_

| **Description**                                   | **Value**     |
| ------------------------------------------------- | ------------- |
| Scheduler handler watermark level in microseconds | 0x00 – 0xFFFF |
|                                                   |

_LHCI Handler Watermark Level (2 Byte):_

| **Description**                              | **Value**     |
| -------------------------------------------- | ------------- |
| LHCI handler watermark level in microseconds | 0x00 – 0xFFFF |
|                                              |

_Max Adv Sets (2 Bytes):_

| **Description**                    | **Value**     |
| ---------------------------------- | ------------- |
| Maximum number of advertising sets | 0x00 – 0xFFFF |
|                                    |

_Adv Set Context Size (2 Bytes):_

| **Description**                          | **Value**     |
| ---------------------------------------- | ------------- |
| Size of advertising set context in bytes | 0x00 – 0xFFFF |
|                                          |

_Ext Scan Max (2 Bytes):_

| **Description**                     | **Value**    |
| ----------------------------------- | ------------ |
| Maximum number of extended scanners | 0x0 – 0xFFFF |

_Ext Scan Context Size (2 Bytes):_

| **Description**                                     | **Value**     |
| --------------------------------------------------- | ------------- |
| Size of context size for extended scanners in bytes | 0x00 – 0xFFFF |
|                                                     |

_Max Num Extended Init (2 Bytes):_

| **Description**                        | **Value**     |
| -------------------------------------- | ------------- |
| maximum number of extended initiators. | 0x00 – 0xFFFF |
|                                        |

_Ext Init Context Size (2 Byte):_

| **Description**                                       | **Value**     |
| ----------------------------------------------------- | ------------- |
| Size of context size for extended initiators in bytes | 0x00 – 0xFFFF |
|                                                       |
_Max Periodic Scanners (2 Bytes):_

| **Description**                     | **Value**   |
| ----------------------------------- | ----------- |
| Maximum number of periodic scanners | 0x00-0xFFFF |
|                                     |

_Periodic Scanners Context Size(2 Bytes):_

| **Description**                            | **Value**   |
| ------------------------------------------ | ----------- |
| Context size of periodic scanners in bytes | 0x00-0xFFFF |
|                                            |

_Max CIG (2 Bytes):_

| **Description**       | **Value**   |
| --------------------- | ----------- |
| Maximum number of CIG | 0x00-0xFFFF |
|                       |

_CIG Context Size (2 Bytes):_

| **Description**              | **Value**   |
| ---------------------------- | ----------- |
| Context size of CIG in bytes | 0x00-0xFFFF |
|                              |
_Max CIS (2 Bytes):_

| **Description**       | **Value**   |
| --------------------- | ----------- |
| Maximum number of CIS | 0x00-0xFFFF |
|                       |


_CIS Context Size (2 Bytes):_

| **Description**              | **Value**   |
| ---------------------------- | ----------- |
| Context size of CIS in bytes | 0x00-0xFFFF |
|                              |
### Get Advertising Stats

| **OCF** | **Length** | **Parameters** |
| ------- | ---------- | -------------- |
| 0x3FB   | 0          | N/A            |
|         |
#### Description

Get the accumulated adverting stats.

#### Return

_TX ADV (4 Bytes):_

| **Description**                    | **Value**         |
| ---------------------------------- | ----------------- |
| Number of sent advertising packets | 0x00 – 0xFFFFFFFF |
|                                    |

_RX Req (4 Bytes):_

| **Description**                                      | **Value**         |
| ---------------------------------------------------- | ----------------- |
| Number of successfully received advertising requests | 0x00 – 0xFFFFFFFF |
|                                                      |


_RX Req CRC (4 Bytes):_

| **Description**                                         | **Value**         |
| ------------------------------------------------------- | ----------------- |
| Number of received advertising requests with CRC errors | 0x00 – 0xFFFFFFFF |
|                                                         |


_RX Req Timeout (4 Bytes):_

| **Description**                                                     | **Value**         |
| ------------------------------------------------------------------- | ----------------- |
| Number of timed out received advertising requests (receive timeout) | 0x00 – 0xFFFFFFFF |
|                                                                     |


_TX RSP (4 Bytes):_

| **Description**                 | **Value**         |
| ------------------------------- | ----------------- |
| Number of sent response packets | 0x00 – 0xFFFFFFFF |
|                                 |


_Err ADV (4 Bytes):_

| **Description**                          | **Value**         |
| ---------------------------------------- | ----------------- |
| Number of advertising transaction errors | 0x00 – 0xFFFFFFFF |
|                                          |


_RX Setup (2 Bytes):_

| **Description**                           | **Value**     |
| ----------------------------------------- | ------------- |
| RX packet setup watermark in microseconds | 0x00 – 0xFFFF |
|                                           |

_TX Setup (2 Bytes):_

| **Description**                           | **Value**     |
| ----------------------------------------- | ------------- |
| TX packet setup watermark in microseconds | 0x00 – 0xFFFF |
|                                           |


_RX ISR (2 Bytes):_

| **Description**                             | **Value**     |
| ------------------------------------------- | ------------- |
| RX ISR processing watermark in microseconds | 0x00 – 0xFFFF |
|                                             |


_TX ISR (2 Bytes):_

| **Description**                             | **Value**     |
| ------------------------------------------- | ------------- |
| TX ISR processing watermark in microseconds | 0x00 – 0xFFFF |
|                                             |


### Get Scan Stats

| **OCF** | **Length** | **Parameters** |
| ------- | ---------- | -------------- |
| 0x3FC   | 0          | N/A            |
|         |
#### Description

Get the statistics captured during scanning.

#### Return

_RX ADV (4 Bytes):_

| **Description**                                     | **Value**         |
| --------------------------------------------------- | ----------------- |
| Number of successfully received advertising packets | 0x00 – 0xFFFFFFFF |
|                                                     |


_RX ADV CRC (4 Bytes):_

| **Description**                                        | **Value**         |
| ------------------------------------------------------ | ----------------- |
| Number of received advertising packets with CRC errors | 0x00 – 0xFFFFFFFF |
|                                                        |


_RX ADV Timeout (4 Bytes):_

| **Description**                                           | **Value**         |
| --------------------------------------------------------- | ----------------- |
| Number of timed out advertising packets (receive timeout) | 0x00 – 0xFFFFFFFF |
|                                                           |


_TX Req (4 Bytes):_

| **Description**                     | **Value**         |
| ----------------------------------- | ----------------- |
| Number of sent advertising requests | 0x00 – 0xFFFFFFFF |
|                                     |


_RX RSP (4 Bytes):_

| **Description**                                              | **Value**         |
| ------------------------------------------------------------ | ----------------- |
| Number of successfully received advertising response packets | 0x00 – 0xFFFFFFFF |
|                                                              |


_RX RSP CRC (4 Bytes):_

| **Description**                                                 | **Value**         |
| --------------------------------------------------------------- | ----------------- |
| Number of received advertising response packets with CRC errors | 0x00 – 0xFFFFFFFF |
|                                                                 |


_RX RSP Timeout (4 Bytes):_

| **Description**                                                    | **Value**         |
| ------------------------------------------------------------------ | ----------------- |
| Number of timed out advertising response packets (receive timeout) | 0x00 – 0xFFFFFFFF |
|                                                                    |


_Err Scan (4 Bytes):_

| **Description**                   | **Value**         |
| --------------------------------- | ----------------- |
| Number of scan transaction errors | 0x00 – 0xFFFFFFFF |
|                                   |


_RX Setup (2 Bytes):_

| **Description**                           | **Value**         |
| ----------------------------------------- | ----------------- |
| RX packet setup watermark in microseconds | 0x00 – 0xFFFFFFFF |
|                                           |


_TX Setup (2 Bytes):_

| **Description**                           | **Value**         |
| ----------------------------------------- | ----------------- |
| TX packet setup watermark in microseconds | 0x00 – 0xFFFFFFFF |
|                                           |

_RX ISR (2 Bytes):_

| **Description**                             | **Value**         |
| ------------------------------------------- | ----------------- |
| RX ISR processing watermark in microseconds | 0x00 – 0xFFFFFFFF |
|                                             |

_TX ISR (2 Bytes):_

| **Description**                             | **Value**         |
| ------------------------------------------- | ----------------- |
| TX ISR processing watermark in microseconds | 0x00 – 0xFFFFFFFF |
|                                             |
### Get Connection Stats

| **OCF** | **Length** | **Parameters** |
| ------- | ---------- | -------------- |
| 0x3FD   | 0          | N/A            |
|         |
#### Description

Get the statistics captured during connection.

#### Return

_RX Data (4 Bytes)_

| **Description**                              | **Value**         |
| -------------------------------------------- | ----------------- |
| Number of successfully received data packets | 0x00 - 0xFFFFFFFF |
|                                              |


_RX Data CRC (4 Bytes)_

| **Description**                                 | **Value**         |
| ----------------------------------------------- | ----------------- |
| Number of received data packets with CRC errors | 0x00 - 0xFFFFFFFF |
|                                                 |


_RX Data Timeout (4 Bytes)_

| **Description**                                    | **Value**         |
| -------------------------------------------------- | ----------------- |
| Number of timed out data packets (receive timeout) | 0x00 - 0xFFFFFFFF |
|                                                    |


_TX Data (4 Bytes)_

| **Description**             | **Value**         |
| --------------------------- | ----------------- |
| Number of sent data packets | 0x00 - 0xFFFFFFFF |
|                             |


_Err Data (4 Bytes)_

| **Description**                   | **Value**         |
| --------------------------------- | ----------------- |
| Number of data transaction errors | 0x00 - 0xFFFFFFFF |
|                                   |

_RX Setup (2 Bytes)_

| **Description**                           | **Value**     |
| ----------------------------------------- | ------------- |
| RX packet setup watermark in microseconds | 0x00 - 0xFFFF |
|                                           |

_TX Setup (2 Bytes)_

| **Description**                           | **Value**     |
| ----------------------------------------- | ------------- |
| TX packet setup watermark in microseconds | 0x00 - 0xFFFF |
|                                           |

_RX ISR (2 Bytes)_

| **Description**                             | **Value**     |
| ------------------------------------------- | ------------- |
| RX ISR processing watermark in microseconds | 0x00 - 0xFFFF |
|                                             |

_TX ISR (2 Bytes)_

| **Description**                             | **Value**     |
| ------------------------------------------- | ------------- |
| TX ISR processing watermark in microseconds | 0x00 - 0xFFFF |
|                                             |


### Get Test Stats

| **OCF** | **Length** | **Parameters** | **Return**                        |
| ------- | ---------- | -------------- | --------------------------------- |
| 0x3FE   | 0          | N/A            | Test stats in order as documented |
|         |
#### Description

Get the statistics captured during test mode.

#### Return

_RX Data (4 Bytes)_

| **Description**                              | **Value**         |
| -------------------------------------------- | ----------------- |
| Number of successfully received data packets | 0x00 - 0xFFFFFFFF |
|                                              |


_RX Data CRC (4 Bytes)_

| **Description**                                 | **Value**         |
| ----------------------------------------------- | ----------------- |
| Number of received data packets with CRC errors | 0x00 - 0xFFFFFFFF |
|                                                 |


_RX Data Timeout (4 Bytes)_

| **Description**                                    | **Value**         |
| -------------------------------------------------- | ----------------- |
| Number of timed out data packets (receive timeout) | 0x00 - 0xFFFFFFFF |
|                                                    |


_TX Data (4 Bytes)_

| **Description**             | **Value**         |
| --------------------------- | ----------------- |
| Number of sent data packets | 0x00 - 0xFFFFFFFF |
|                             |


_Err Data (4 Bytes)_

| **Description**                   | **Value**         |
| --------------------------------- | ----------------- |
| Number of data transaction errors | 0x00 - 0xFFFFFFFF |
|                                   |

_RX Setup (2 Bytes)_

| **Description**                           | **Value**     |
| ----------------------------------------- | ------------- |
| RX packet setup watermark in microseconds | 0x00 - 0xFFFF |
|                                           |


_TX Setup (2 Bytes)_

| **Description**                           | **Value**     |
| ----------------------------------------- | ------------- |
| TX packet setup watermark in microseconds | 0x00 - 0xFFFF |
|                                           |


_RX ISR (2 Bytes)_

| **Description**                             | **Value**     |
| ------------------------------------------- | ------------- |
| RX ISR processing watermark in microseconds | 0x00 - 0xFFFF |
|                                             |


_TX ISR (2 Bytes)_

| **Description**                             | **Value**     |
| ------------------------------------------- | ------------- |
| TX ISR processing watermark in microseconds | 0x00 - 0xFFFF |
|                                             |

### Get Pool Stats

| **OCF** | **Length** | **Parameters** | **Return**                        |
| ------- | ---------- | -------------- | --------------------------------- |
| 0x3FF   | 0          | N/A            | Pool stats in order as documented |
|         |

#### Description

Get the memory pool statistics captured during runtime.

**NOTE:** The flag _WSF\_BUF\_STATS_ must be defined to _TRUE_ at compile time

#### Return

_Num Pool (1 Bytes):_

| **Description**         | **Value**   |
| ----------------------- | ----------- |
| Number of pools defined | 0x00 – 0xFF |
|                         |

**Note:** The rest of the return parameters may be repeated _Num Pool_ times and will be sent in order of the pool number (e.g., pool 0, pool 1, …, pool N-1).

_Buf Size (2 Bytes):_

| **Description**  | **Value**     |
| ---------------- | ------------- |
| Pool Buffer Size | 0x00 – 0xFFFF |
|                  |

_Num Buf (1 Byte):_

| **Description**         | **Value**   |
| ----------------------- | ----------- |
| Total Number of buffers | 0x00 – 0xFF |
|                         |


_Num Alloc (1 Byte):_

| **Description**                   | **Value**   |
| --------------------------------- | ----------- |
| Number of outstanding allocations | 0x00 – 0xFF |
|                                   |

_Max Alloc (1 Byte):_

| **Description**           | **Value**   |
| ------------------------- | ----------- |
| High allocation watermark | 0x00 – 0xFF |
|                           |


_Max Req Len (2 Bytes):_

| **Description**                 | **Value**     |
| ------------------------------- | ------------- |
| Maximum requested buffer length | 0x00 – 0xFFFF |
|                                 |


### Set Additional AuxPtr Offset

| **OCF** | **Length** | **Parameters** |
| ------- | ---------- | -------------- |
| 0x3D0   | 5          | Delay, Handle  |
|         |


#### Description

Set auxiliary packet offset delay.

#### Parameters

_Delay (4 Bytes):_

| **Description**       | **Value**        |
| --------------------- | ---------------- |
| Disable               | 0x00             |
| Delay in microseconds | 0x1 – 0xFFFFFFFF |
|                       |

_Handle (1 Byte):_

| **Description**   | **Value**                     |
| ----------------- | ----------------------------- |
| Connection handle | 0x01-MAX\_NUMBER\_CONNECTIONS |
|                   |


#### Return

Status

### Set Extended Advertising Data Fragmentation

| **OCF** | **Length** | **Parameters**      |
| ------- | ---------- | ------------------- |
| 0x3D1   | 2          | Handle, Frag Length |
|         |


#### Description

Set the extended advertising fragmentation length.

#### Parameters

_Handle (1 Bytes):_

| **Description**    | **Value**                     |
| ------------------ | ----------------------------- |
| Advertising Handle | 0x01-MAX\_NUMBER\_CONNECTIONS |
|                    |


_Frag Length (1 Bytes):_

| **Description**      | **Value** |
| -------------------- | --------- |
| Fragmentation Length | 0x00-0xFF |

#### Return

Status

### Set Extended Advertising PHY Options

| **OCF** | **Length** | **Parameters**                       |
| ------- | ---------- | ------------------------------------ |
| 0x3D2   | 3          | Handle, Primary Opt., Secondary Opt. |
|         |


#### Description

Set extended advertising PHY options

#### Parameters

_Handle (1 Byte):_

| **Description**    | **Value**                     |
| ------------------ | ----------------------------- |
| Advertising Handle | 0x01-MAX\_NUMBER\_CONNECTIONS |
|                    |

_Primary Opt. (1 Byte):_

| **Description**                          | **Value** |
| ---------------------------------------- | --------- |
| Primary advertising channel PHY options. | 0x00-0xFF |

_Secondary Opt. (1 Byte):_

| **Description**                            | **Value** |
| ------------------------------------------ | --------- |
| Secondary advertising channel PHY options. | 0x00-0xFF |

#### Return

Status

### Set extended Advertising Default PHY Options

| **OCF** | **Length** | **Parameters** |
| ------- | ---------- | -------------- |
| 0x3D3   | 1          | PHY Opt.       |
|         |


#### Description

Set the default TX PHY options for extended adv slave primary and secondary channel.

#### Parameters

_PHY Opt. (1 Byte):_

| **Description** | **Value**   |
| --------------- | ----------- |
| PHY Options     | 0x00 – 0xFF |
|                 |

#### Return

Status

### Generate ISO Packets

| **OCF** | **Length** | **Parameters**                     |
| ------- | ---------- | ---------------------------------- |
| 0x3D5   | 5          | Handle, Packet Length, Num Packets |
|         |

#### Description

Generate ISO packets.

#### Parameters

_Handle (2 Byte):_

| **Description**   | **Value**                     |
| ----------------- | ----------------------------- |
| Connection Handle | 0x01-MAX\_NUMBER\_CONNECTIONS |
|                   |

_Packet Length (2 Byte):_

| **Description** | **Value**   |
| --------------- | ----------- |
| Packet Length   | 0x00-0xFFFF |
|                 |

_Num Packets (1 Byte):_

| **Description**   | **Value** |
| ----------------- | --------- |
| Number of packets | 0x00-0xFF |
|                   |
#### Return

Status

### Get ISO Test Report

| **OCF** | **Length** | **Parameters**                                                                   |
| ------- | ---------- | -------------------------------------------------------------------------------- |
| 0x3D6   | 16         | RX ISO Packet Count,RX ISO Octet CountGenerate Packet Count,Generate Octet Count |
|         |

#### Description

Get statistics captured during ISO test.

#### Return

_RX ISO Packet Count (4 Byte):_

| **Description**          | **Value**         |
| ------------------------ | ----------------- |
| Receive ISO Packet Count | 0x00 – 0xFFFFFFFF |
|                          |

_RX ISO Octet Count (4 Byte):_

| **Description**         | **Value**         |
| ----------------------- | ----------------- |
| Receive ISO Octet Count | 0x00 – 0xFFFFFFFF |
|                         |

_Generate Packet Count (4 Byte):_

| **Description**           | **Value**         |
| ------------------------- | ----------------- |
| Generate ISO Packet Count | 0x00 – 0xFFFFFFFF |
|                           |

_Generate Octet Count (4 Byte):_

| **Description**          | **Value**         |
| ------------------------ | ----------------- |
| Generate ISO Octet Count | 0x00 – 0xFFFFFFFF |
|                          |

### Enable ISO Packet Sink

| **OCF** | **Length** | **Parameters** |
| ------- | ---------- | -------------- |
| 0x3D7   | 1          | Enable         |
|         |

#### Description

Enable/Disable ISO packet sink.

#### Parameters

_Enable (1 Byte):_

| **Description** | **Value** |
| --------------- | --------- |
| Disable         | 0x00      |
| Enable          | 0x01      |
|                 |

#### Return

Status

### Enable Autogenerate ISO Packets

| **OCF** | **Length** | **Parameters** |
| ------- | ---------- | -------------- |
| 0x3D8   | 2          | Packet Length  |
|         |

#### Description

Enable autogenerate ISO packets.

#### Parameters

_Packet Length (2 Bytes):_

| **Description** | **Value**     |
| --------------- | ------------- |
| Disable         | 0x00          |
| Length          | 0x01 – 0xFFFF |
|                 |

#### Return

Status

### Get ISO Connection Statistics

| **OCF** | **Length** | **Parameters** |
| ------- | ---------- | -------------- |
| 0x3D9   | 0          | N/A            |
|         |
#### Description

Get statistics captured during ISO connection.

#### Return

_RX Data (4 Bytes)_

| **Description**                              | **Value**         |
| -------------------------------------------- | ----------------- |
| Number of successfully received data packets | 0x00 - 0xFFFFFFFF |
|                                              |

_RX Data CRC (4 Bytes)_

| **Description**                                 | **Value**         |
| ----------------------------------------------- | ----------------- |
| Number of received data packets with CRC errors | 0x00 - 0xFFFFFFFF |
|                                                 |

_RX Data Timeout (4 Bytes)_

| **Description**                                    | **Value**         |
| -------------------------------------------------- | ----------------- |
| Number of timed out data packets (receive timeout) | 0x00 - 0xFFFFFFFF |
|                                                    |

_TX Data (4 Bytes)_

| **Description**             | **Value**         |
| --------------------------- | ----------------- |
| Number of sent data packets | 0x00 - 0xFFFFFFFF |
|                             |

_Err Data (4 Bytes)_

| **Description**                   | **Value**         |
| --------------------------------- | ----------------- |
| Number of data transaction errors | 0x00 - 0xFFFFFFFF |
|                                   |

_RX Setup (2 Bytes)_

| **Description**                           | **Value**     |
| ----------------------------------------- | ------------- |
| RX packet setup watermark in microseconds | 0x00 - 0xFFFF |
|                                           |

_TX Setup (2 Bytes)_

| **Description**                           | **Value**     |
| ----------------------------------------- | ------------- |
| TX packet setup watermark in microseconds | 0x00 - 0xFFFF |
|                                           |

_RX ISR (2 Bytes)_

| **Description**                             | **Value**     |
| ------------------------------------------- | ------------- |
| RX ISR processing watermark in microseconds | 0x00 - 0xFFFF |
|                                             |


_TX ISR (2 Bytes)_

| **Description**                             | **Value**     |
| ------------------------------------------- | ------------- |
| TX ISR processing watermark in microseconds | 0x00 - 0xFFFF |
|                                             |
### Get Auxiliary Advertising Statistics

| **OCF** | **Length** | **Parameters** |
| ------- | ---------- | -------------- |
| 0x3DA   | 0          | N/A            |
|         |
#### Description

Get accumulated auxiliary advertising stats.

#### Return

_TX ADV (4 Bytes):_

| **Description**                    | **Value**         |
| ---------------------------------- | ----------------- |
| Number of sent advertising packets | 0x00 – 0xFFFFFFFF |
|                                    |

_RX Req (4 Bytes):_

| **Description**                                      | **Value**         |
| ---------------------------------------------------- | ----------------- |
| Number of successfully received advertising requests | 0x00 – 0xFFFFFFFF |
|                                                      |

_RX Req CRC (4 Bytes):_

| **Description**                                         | **Value**         |
| ------------------------------------------------------- | ----------------- |
| Number of received advertising requests with CRC errors | 0x00 – 0xFFFFFFFF |
|                                                         |

_RX Req Timeout (4 Bytes):_

| **Description**                                                     | **Value**         |
| ------------------------------------------------------------------- | ----------------- |
| Number of timed out received advertising requests (receive timeout) | 0x00 – 0xFFFFFFFF |
|                                                                     |


_TX RSP (4 Bytes):_

| **Description**                 | **Value**         |
| ------------------------------- | ----------------- |
| Number of sent response packets | 0x00 – 0xFFFFFFFF |
|                                 |

_TX Chain (4 Bytes):_

| **Description**              | **Value**         |
| ---------------------------- | ----------------- |
| Number of sent chain packets | 0x00 – 0xFFFFFFFF |
|                              |

_Err ADV (4 Bytes):_

| **Description**                          | **Value**         |
| ---------------------------------------- | ----------------- |
| Number of advertising transaction errors | 0x00 – 0xFFFFFFFF |
|                                          |

_RX Setup (2 Bytes):_

| **Description**                           | **Value**     |
| ----------------------------------------- | ------------- |
| RX packet setup watermark in microseconds | 0x00 – 0xFFFF |
|                                           |

_TX Setup (2 Bytes):_

| **Description**                           | **Value**     |
| ----------------------------------------- | ------------- |
| TX packet setup watermark in microseconds | 0x00 – 0xFFFF |
|                                           |

_RX ISR (2 Bytes):_

| **Description**                             | **Value**     |
| ------------------------------------------- | ------------- |
| RX ISR processing watermark in microseconds | 0x00 – 0xFFFF |
|                                             |

_TX ISR (2 Bytes):_

| **Description**                             | **Value**     |
| ------------------------------------------- | ------------- |
| TX ISR processing watermark in microseconds | 0x00 – 0xFFFF |
|                                             |
 

### Get Auxiliary Scanning Statistics

| **OCF** | **Length** | **Parameters** |
| ------- | ---------- | -------------- |
| 0x3DB   | 0          | N/A            |
|         |
#### Description

Get accumulated auxiliary scanning statistics.

#### Return

_RX ADV (4 Bytes):_

| **Description**                    | **Value**         |
| ---------------------------------- | ----------------- |
| Number of sent advertising packets | 0x00 – 0xFFFFFFFF |
|                                    |

_RX ADV CRC (4 Bytes):_

| **Description**                                         | **Value**         |
| ------------------------------------------------------- | ----------------- |
| Number of received advertising requests with CRC errors | 0x00 – 0xFFFFFFFF |
|                                                         |

_RX ADV Timeout (4 Bytes):_

| **Description**                                                     | **Value**         |
| ------------------------------------------------------------------- | ----------------- |
| Number of timed out received advertising requests (receive timeout) | 0x00 – 0xFFFFFFFF |
|                                                                     |

_TX REQ (4 Bytes):_

| **Description**                     | **Value**         |
| ----------------------------------- | ----------------- |
| Number of sent advertising requests | 0x00 – 0xFFFFFFFF |
|                                     |

_RX RSP (4 Bytes):_

| **Description**                                              | **Value**         |
| ------------------------------------------------------------ | ----------------- |
| Number of successfully received advertising response packets | 0x00 – 0xFFFFFFFF |
|                                                              |

_RX RSP CRC (4 Bytes):_

| **Description**                                                 | **Value**         |
| --------------------------------------------------------------- | ----------------- |
| Number of received advertising response packets with CRC errors | 0x00 – 0xFFFFFFFF |
|                                                                 |

_RX RSP Timeout (4 Bytes):_

| **Description**                                                    | **Value**         |
| ------------------------------------------------------------------ | ----------------- |
| Number of timed out advertising response packets (receive timeout) | 0x00 – 0xFFFFFFFF |
|                                                                    |


_RX Chain (4 Bytes):_

| **Description**                               | **Value**         |
| --------------------------------------------- | ----------------- |
| Number of successfully received chain packets | 0x00 – 0xFFFFFFFF |
|                                               |


_RX Chain CRC (4 Bytes):_

| **Description**                                  | **Value**         |
| ------------------------------------------------ | ----------------- |
| Number of received chain packets with CRC errors | 0x00 – 0xFFFFFFFF |

_RX Chain Timeout (4 Bytes):_

| **Description**                                     | **Value**         |
| --------------------------------------------------- | ----------------- |
| Number of timed out chain packets (receive timeout) | 0x00 – 0xFFFFFFFF |

_Err Scan (4 Bytes):_

| **Description**                   | **Value**         |
| --------------------------------- | ----------------- |
| Number of scan transaction errors | 0x00 – 0xFFFFFFFF |
|                                   |

_RX Setup (2 Bytes):_

| **Description**                           | **Value**     |
| ----------------------------------------- | ------------- |
| RX packet setup watermark in microseconds | 0x00 – 0xFFFF |
|                                           |


_TX Setup (2 Bytes):_

| **Description**                           | **Value**     |
| ----------------------------------------- | ------------- |
| TX packet setup watermark in microseconds | 0x00 – 0xFFFF |
|                                           |

_RX ISR (2 Bytes):_

| **Description**                             | **Value**     |
| ------------------------------------------- | ------------- |
| RX ISR processing watermark in microseconds | 0x00 – 0xFFFF |
|                                             |

_TX ISR (2 Bytes):_

| **Description**                             | **Value**     |
| ------------------------------------------- | ------------- |
| TX ISR processing watermark in microseconds | 0x00 – 0xFFFF |
|                                             |


### Get Periodic Scanning Statistics

| **OCF** | **Length** | **Parameters** |
| ------- | ---------- | -------------- |
| 0x3DC   | 0          | N/A            |

#### Description

Get accumulated periodic scanning statistics.

#### Return

_RX ADV (4 Bytes):_

| **Description**                                     | **Value**         |
| --------------------------------------------------- | ----------------- |
| Number of successfully received advertising packets | 0x00 – 0xFFFFFFFF |
|                                                     |

_RX ADV CRC (4 Bytes):_

| **Description**                                        | **Value**         |
| ------------------------------------------------------ | ----------------- |
| Number of received advertising packets with CRC errors | 0x00 – 0xFFFFFFFF |

_RX ADV Timeout (4 Bytes):_

| **Description**                                           | **Value**         |
| --------------------------------------------------------- | ----------------- |
| Number of timed out advertising packets (receive timeout) | 0x00 – 0xFFFFFFFF |

_RX Chain (4 Bytes):_

| **Description**                               | **Value**         |
| --------------------------------------------- | ----------------- |
| Number of successfully received chain packets | 0x00 – 0xFFFFFFFF |

_RX Chain CRC (4 Bytes):_

| **Description**                                  | **Value**         |
| ------------------------------------------------ | ----------------- |
| Number of received chain packets with CRC errors | 0x00 – 0xFFFFFFFF |

_RX Chain Timeout (4 Bytes):_

| **Description**                                     | **Value**         |
| --------------------------------------------------- | ----------------- |
| Number of timed out chain packets (receive timeout) | 0x00 – 0xFFFFFFFF |

_Err Scan (4 Bytes):_

| **Description**                   | **Value**         |
| --------------------------------- | ----------------- |
| Number of scan transaction errors | 0x00 – 0xFFFFFFFF |

_RX Setup (2 Bytes):_

| **Description**                           | **Value**     |
| ----------------------------------------- | ------------- |
| RX packet setup watermark in microseconds | 0x00 – 0xFFFF |

_TX Setup (2 Bytes):_

| **Description**                           | **Value**     |
| ----------------------------------------- | ------------- |
| TX packet setup watermark in microseconds | 0x00 – 0xFFFF |

_RX ISR (2 Bytes):_

| **Description**                             | **Value**     |
| ------------------------------------------- | ------------- |
| RX ISR processing watermark in microseconds | 0x00 – 0xFFFF |

_TX ISR (2 Bytes):_

| **Description**                             | **Value**     |
| ------------------------------------------- | ------------- |
| TX ISR processing watermark in microseconds | 0x00 – 0xFFFF |

### Set Connection PHY TX Power

| **OCF** | **Length** | **Parameters**     |
| ------- | ---------- | ------------------ |
| 0x3DD   | 4          | Handle, Level, PHY |

#### Description

Set power level during a connection for a given PHY.

#### Parameters

_Handle (2 Bytes):_

| **Description**   | **Value**                     |
| ----------------- | ----------------------------- |
| Connection handle | 0x01-MAX\_NUMBER\_CONNECTIONS |

_Level (1 Byte, Signed 8-Bit):_

| **Description** | **Value** |
| --------------- | --------- |
| Power Level     | -15 - 6   |
|                 |

_PHY (1 Byte, Signed 8-Bit):_

| **Description** | **Value** |
| --------------- | --------- |
| 1M              | 0x00      |
| 2M              | 0x01      |
| Coded           | 0x02      |
|                 |


#### Return

Status