/**
* @file     i3c_ccc.h
* @brief    Improved Inter Integrated Circuit (I3C) Common Command Codes (CCC).
*/

/******************************************************************************
 *
 * Copyright (C) 2024 Analog Devices, Inc.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 ******************************************************************************/

#ifndef LIBRARIES_PERIPHDRIVERS_SOURCE_I3C_I3C_CCC_H_
#define LIBRARIES_PERIPHDRIVERS_SOURCE_I3C_I3C_CCC_H_

/* Broadcast CCCs */
#define MXC_I3C_CCC_B_ENEC 0x00 /* Enable Events Command */
#define MXC_I3C_CCC_B_DISEC 0x01 /* Disable Events Command */
#define MXC_I3C_CCC_B_ENTAS0 0x02 /* Enter Activity State 0 */
#define MXC_I3C_CCC_B_ENTAS1 0x03 /* Enter Activity State 1 */
#define MXC_I3C_CCC_B_ENTAS2 0x04 /* Enter Activity State 2 */
#define MXC_I3C_CCC_B_ENTAS3 0x05 /* Enter Activity State 3 */
#define MXC_I3C_CCC_B_RSTDAA 0x06 /* Reset Dynamic Address Assignment */
#define MXC_I3C_CCC_B_ENTDAA 0x07 /* Enter Dynamic Address Assignment */
#define MXC_I3C_CCC_B_DEFTGTS 0x08 /* Define List of Targets */
#define MXC_I3C_CCC_B_SETMWL 0x09 /* Set Max Write Length */
#define MXC_I3C_CCC_B_SETMRL 0x0A /* Set Max Read Length */
#define MXC_I3C_CCC_B_ENTTM 0x0B /* Enter Test Mode */
#define MXC_I3C_CCC_B_SETBUSCON 0x0C /* Set Bus Context */
#define MXC_I3C_CCC_B_ENDXFR 0x12 /* Data Transfer Ending Procedure Control */
#define MXC_I3C_CCC_B_ENTHDR0 0x20 /* Enter HDR Mode 0 */
#define MXC_I3C_CCC_B_ENTHDR1 0x21 /* Enter HDR Mode 1 */
#define MXC_I3C_CCC_B_ENTHDR2 0x22 /* Enter HDR Mode 2 */
#define MXC_I3C_CCC_B_ENTHDR3 0x23 /* Enter HDR Mode 3 */
#define MXC_I3C_CCC_B_ENTHDR4 0x24 /* Enter HDR Mode 4 */
#define MXC_I3C_CCC_B_ENTHDR5 0x25 /* Enter HDR Mode 5 */
#define MXC_I3C_CCC_B_ENTHDR6 0x26 /* Enter HDR Mode 6 */
#define MXC_I3C_CCC_B_ENTHDR7 0x27 /* Enter HDR Mode 7 */
#define MXC_I3C_CCC_B_SETXTIME 0x28 /* Exchange Timing Information */
#define MXC_I3C_CCC_B_SETAASA 0x29 /* Set Dynamic Address as Static Address */
#define MXC_I3C_CCC_B_RSTACT 0x2A /* Target Reset Action */
#define MXC_I3C_CCC_B_DEFGRPA 0x2B /* Define List of Group Addresses */
#define MXC_I3C_CCC_B_RSTGRPA 0x2C /* Reset Group Address */
#define MXC_I3C_CCC_B_MLANE 0x2D /* Multi-Land Data Transfer Control */

/* Direct CCCs */
#define MXC_I3C_CCC_D_ENEC 0x80 /* Enable Events Command */
#define MXC_I3C_CCC_D_DISEC 0x81 /* Disable Events Command */
#define MXC_I3C_CCC_D_ENTAS0 0x82 /* Enter Activity State 0 */
#define MXC_I3C_CCC_D_ENTAS1 0x83 /* Enter Activity State 1 */
#define MXC_I3C_CCC_D_ENTAS2 0x84 /* Enter Activity State 2 */
#define MXC_I3C_CCC_D_ENTAS3 0x85 /* Enter Activity State 3 */
#define MXC_I3C_CCC_D_RSTDAA 0x86 /* Reset Dynamic Address Assignment NACKed */
#define MXC_I3C_CCC_D_SETDASA 0x87 /* Set Dynamic Address from Static Address */
#define MXC_I3C_CCC_D_SETNEWDA 0x88 /* Set New Dynamic Address */
#define MXC_I3C_CCC_D_SETMWL 0x89 /* Set Max Write Length */
#define MXC_I3C_CCC_D_SETMRL 0x8A /* Set Max Read Length */
#define MXC_I3C_CCC_D_GETMWL 0x8B /* Get Max Write Length */
#define MXC_I3C_CCC_D_GETMRL 0x8C /* Get Max Read Length */
#define MXC_I3C_CCC_D_GETPID 0x8D /* Get Provisioned ID */
#define MXC_I3C_CCC_D_GETBCR 0x8E /* Get Bus Characteristics Register */
#define MXC_I3C_CCC_D_GETDCR 0x8F /* Get Device Characteristics Register */
#define MXC_I3C_CCC_D_GETSTATUS 0x90 /* Get Device Status */
#define MXC_I3C_CCC_D_GETACCCR 0x91 /* Get Accept Controller Role */
#define MXC_I3C_CCC_D_ENDXFR 0x92 /* Data Transfer Ending Procedure Control */
#define MXC_I3C_CCC_D_SETBRGTGT 0x93 /* Set Bridge Targets */
#define MXC_I3C_CCC_D_GETMXDS 0x94 /* Get Max Data Speed */
#define MXC_I3C_CCC_D_GETCAPS 0x95 /* Get Optional Feature Capabilities */
#define MXC_I3C_CCC_D_D2DXFER 0x97 /* Device to Device(s) Tunneling Control */
#define MXC_I3C_CCC_D_SETXTIME 0x98 /* Set Exchange Timing Information */
#define MXC_I3C_CCC_D_GETXTIME 0x99 /* Get Exchange Timing Information */
#define MXC_I3C_CCC_D_RSTACT 0x9A /* Target Reset Action */
#define MXC_I3C_CCC_D_SETGRPA 0x9B /* Set Group Address */
#define MXC_I3C_CCC_D_RSTGRPA 0x9C /* Reset Group Address */
#define MXC_I3C_CCC_D_MLANE 0x9D /* Multi-Lane Data Transfer Control */

#endif // LIBRARIES_PERIPHDRIVERS_SOURCE_I3C_I3C_CCC_H_
