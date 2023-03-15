# Cordio Bluetooth LE User Guide

This library implements the Bluetooth Low Energy stack, forked from [Packetcraft](https://github.com/packetcraft-inc/stacks). It depends on additional libraries within the msdk repository. All applications utilizing this library can be found in the `Examples` directory of an MSDK installation.

## Supported Features

* **LE Dual Mode Topology:** Advertise/Scan while maintaining multiple adjacent connections.
* **2M PHY:** High-speed PHY with 2 Mbps symbol rate.
* **Coded PHY:** Forward error correction, 125 kbps or 500 kbps symbol rate.
* **Extended Advertising:** Enhanced advertising and scanning.
* **Periodic Advertising:** Broadcasting and receiving periodic advertisements.
* **EATT:** Enhanced Attribute protocol.
* **LE Secure Connections:** LE Secure Connections is an enhanced security feature introduced in Bluetooth v4.2. It uses a Federal Information Processing Standards (FIPS) compliant algorithm called Elliptic Curve Diffie Hellman (ECDH) for key generation.
* **Data Length Extension:** Extend the maximum data length supported in a connection.
* **LE Power Control:** Dynamic TX power control.
* **\*OTA firmware update:** Over-the-air firmware update is not a standard Bluetooth feature. We have a wireless data exchange service that can be used for OTA firmware updates. See the BLE_otac_otas examples, the ADI Attach smartphone app, and [BLE-PyDex](https://github.com/EdwinFairchild/BLE-PyDex) for details.

### Features in Development

* **Direction finding:** Detect the direction of the incoming signal. Also known as Angle of Arrival (AoA) and Angle of Departure (AoD). Hardware support on MAX32655 and MAX32690, unavailable on MAX32665, software in development.
* **LE Audio:** Isochronous audio with Bluetooth LE. Hardware support on all platforms, software in development.

## Platforms

This release was tested on the following platforms. Note: platforms listed may not be available in this repository.

* Nordic nRF52840 / PCA10056 development kit / Nordic nRF5 SDK 16.0.0 (make configuration: "PLATFORM=nordic BOARD=PCA10056")
* Nordic nRF52832 / PCA10040 development kit / Nordic nRF5 SDK 16.0.0 (make configuration: "PLATFORM=nordic BOARD=PCA10040")
* Laird BL654 / 451-00004 USB adapter / Nordic nRF5 SDK 16.0.0 (make configuration: "PLATFORM=laird")

## Getting Started

The best application to get started with is the [BLE_periph](../docs/Applications/BLE_periph.md) application. It is a simple peripheral application that will allow you to advertise and connect with a central device such as a smartphone.

Follow the MSDK [User Guide](../../../USERGUIDE.md) instructions to install the necessary tools and create a new project. Build and run the `BLE_periph` application for the appropriate target.

## Bluetooth LE Basics

### References

The Bluetooth Special Interest Group (SIG) has documentation available on its [website](https://www.bluetooth.com/). For the latest core specification, refer to the [documentation](https://www.bluetooth.com/specifications/specs/) page. This will be your best resource for learning the details of Bluetooth Low Energy.

These books are also excellent references for developers.

* *Getting Started with Bluetooth Low Energy* by O'Reilly
* *Bluetooth Low Energy: The Developer's Handbook* by Robin Heydon

### Architecture

The Bluetooth stack closely resembles the layers of the network stack. We have the application layer at the top and the physical layer at the bottom. Each layer encapsulates the data and passes it to the appropriate section of the upper and lower layers.

![Stack](res/Stack.PNG)

The Host Controller Interface (HCI) is the common point where devices are split. Typically this interface is over an asynchronous protocol such as UART. Some devices will define proprietary interfaces between the application and host layers. Multi-core SOCs can also use the HCI or proprietary interfaces to split the stack between multiple CPUs.


Test equipment will have a USB interface and act as a Host device when testing the Controller layers. Devices under test will use a USB to UART adapter and act as Controller devices. 

![HCI](res/HCI.PNG)

### States

These are the common states used in Bluetooth LE communication. Typically devices will be in only one of these states at a time, but it is possible for devices to be in all simultaneously. 

#### Advertising

Devices in this state are broadcasting advertisement packets to scanning/initiating devices. This is an asynchronous operation that has no synchronization with peer devices. Advertising devices transmit without any previous knowledge of peer devices. Advertising and scanning operations are done on channels 37, 38, and 39. The interval between advertising events is configurable between 20 ms and 10.24 s.

![ADV Scan](res/ADV_SCN.png)

#### Scanning / Initiating

Devices in the scanning state listen for advertising devices and can send scan optionally requests for additional information. The scanning interval and window settings are configurable. 

If a device is scanning with the intent of connecting to a specific device, that is called the initiating state. The initiating devices will send a connection indication to the desired advertising device to indicate its desire to create a connection.

#### Connected

Once an initiating device sends the connection request and the advertising device accepts the connection request, the two devices enter the connected state. This is a point-to-point connection allowing devices to exchange information directly.

In order to minimize interference, devices in the connected state will hop between channels 0-36 in a pseudo-random order. The channel hopping information is communicated in the connection indication.

Each connection event is separated by the connection interval. This interval is configurable from 7.5 ms to 4 s. The master will always transmit first and receive second. The Slave will always receive first and transmit second. Devices will typically always send and receive at least one packet in each interval, and they can optionally transmit and receive multiple packets in each interval.

![Connected](res/Connected.png)

## Cordio Stack Architecture

This document describes the Cordio software architecture. Refer to the Bluetooth specification Volume 1, Part A for additional information regarding the Bluetooth architecture.

![Software Stack](res/Software_Stack.jpg)

### Application

The App Framework performs many operations common to Bluetooth LE embedded applications, such as:

* Application-level device, connection, and security management.
* Simple user interface abstractions for button press handling, sounds, display, and other user feedback.
* An abstracted device database for storing bonding data and other device parameters.

### Profiles and Services

The GATT Profile specifies the structure in which profile data is exchanged. This structure defines basic elements, such as services and characteristics, used in a profile. The top level of the hierarchy is a profile. A profile is composed of one or more services necessary to fulfill a use case. A service is composed of characteristics or references to other services. Each characteristic contains a value and may contain optional information about the value. The service and characteristic and the components of the characteristic (i.e., value and descriptors) contain the profile data and are all stored in Attributes on the server.

![Profiles](res/Profiles.PNG)

### Wireless Stack Framework

The Wireless Software Foundation (WSF) is a simple OS wrapper, porting layer, and general-purpose software service used by the software system. The goal of WSF is to stay small and lean, supporting only the basic services required by the stack. It consists of the following:

* Event handler service with event and message passing.
* Timer service.
* Queue and buffer management service.
* Portable data types.
* Critical sections and task locking.
* Trace and assert diagnostic services.
* Security interfaces for encryption and random number generation.

#### Platform Adaption Layer

The Platform Adaption Layer is the abstraction between the software stack and the hardware. It includes APIs for timers, UART, RTC, and various system-level functions such as sleep and memory management. 

### Attribute Protocol

The ATT subsystem implements the attribute protocol and generic attribute profile (GATT). It contains two independent subsystems: The attribute protocol client (ATTC) and the attribute protocol server (ATTS).

ATTC implements all attribute protocol client features and is designed to meet the client requirements of the generic attribute profile. ATTC can support multiple simultaneous connections to different servers.

ATTS implements all attribute protocol server features and has support for multiple simultaneous client connections. ATTS also implements the server features defined by the generic attribute profile.

### Device Manager

The DM subsystem implements device management procedures required by the stack. These procedures are partitioned by procedure category and device role (master or slave). The following procedures are implemented in DM:

* Advertising and device visibility: Enable/disable advertising, set advertising parameters and data, and set connectivity and discoverability.
* Scanning and device discovery: Start/stop scanning, set scan parameters, advertising reports, and name discovery.
* Connection management: Create/accept/remove connections, set/update connection parameters, and read RSSI.
* Security management: Bonding, storage of security parameters, authentication, encryption, authorization, random address management.
* Local device management: Initialization and reset, set local parameters, vendor-specific commands.
DM procedures support the Generic Access Profile (GAP) when applicable.

### Security Manager Protocol

The Security Manager Protocol (SMP) is the peer-to-peer protocol used to generate encryption keys and identity keys. The protocol operates over a dedicated fixed L2CAP channel. The SMP block also manages the storage of the encryption keys and identity keys and is responsible for generating random addresses and resolving random addresses to known device identities. The SMP block interfaces directly with the Controller to provide stored keys used for encryption and authentication during the encryption or pairing procedures.

The SMP subsystem implements the security manager protocol. It contains two independent subsystems:

* The initiator (SMPI). SMPI implements the initiator features of the security manager protocol and has support for multiple simultaneous connections.
* The responder (SMPR). SMPR implements the responder features of the security manager protocol and has support for only one connection (by Bluetooth specification design).

SMP also implements the cryptographic toolbox, which uses AES. The interface to AES is asynchronous and abstracted through WSF. SMP also implements functions to support data signing.

### Logical Link Control Adaptation Protocol

The L2CAP (Logical Link Control Adaptation Protocol) resource manager block is responsible for managing the ordering of submission of PDU fragments to the baseband and some relative scheduling between channels to ensure that L2CAP channels with QoS commitments are not denied access to the physical channel due to Controller resource exhaustion. This is required because the architectural model does not assume that a Controller has limitless buffering or that the HCI is a pipe of infinite bandwidth.

L2CAP Resource Managers may also carry out traffic conformance policing to ensure that applications are submitting L2CAP SDUs within the bounds of their negotiated QoS settings. The general Bluetooth data transport model assumes well-behaved applications and does not define how an implementation is expected to deal with this problem.

### Host Controller Interface

The HCI subsystem implements the host-controller interface specification. This specification defines commands, events, and data packets sent between a Bluetooth LE protocol stack on a host and a link layer on a controller.
The HCI API is optimized to be a thin interface layer for a single-chip system. It is configurable for either a single-chip system or a traditional system with wired HCI.
This reconfigurability is accomplished through a layered implementation. A core layer can be configured for either a single-chip system or wired HCI. A transport and driver layer below the core layer can be configured for different wired transports such as UART.

### Link Layer

The link layer is responsible for the creation, modification, and release of logical links (and, if required, their associated logical transports), as well as the update of parameters related to physical links between devices. The link layer achieves this by communicating with the link layer in remote Bluetooth devices using the  Link Layer Protocol (LL) in LE. The LL protocol allows the creation of new logical links and logical transports between devices when required, as well as the general control of link and transport attributes such as the enabling of encryption on the logical transport and the adapting of transmit power on the physical link.

### Physical Layer

The PHY block is responsible for transmitting and receiving packets of information on the physical channel. A control path between the baseband and the PHY block allows the baseband block to control the timing and frequency carrier of the PHY block. The PHY block transforms a stream of data to and from the physical channel and the baseband into required formats.

## ADI Attach

ADI Attach is a smartphone application that can be used for Bluetooth debugging and development.

* Scan for advertising peripherals.
* Connect to devices and discover profiles and services.
* Read and write characteristics.
* Subscribe to notifications.
* Perform over-the-air firmware updates with supporting devices.

## BLE-PyDex

BLE-PyDex is a hardware-agnostic Bluetooth device explorer designed to aid in the development and debugging of Bluetooth applications.

[https://github.com/EdwinFairchild/BLE-PyDex](https://github.com/EdwinFairchild/BLE-PyDex)

## Frequently asked questions

### How do I change the advertising parameters?

Peripheral applications will have a static structure that contains all of the advertising parameters. If run-time changes are desired, you must call ```AppAdvStop()``` before changing the parameters and ```AppAdvStart()``` to resume.


With this configuration, the device will advertise at a fast interval (300 * 0.625 = 187.5 ms) for 5 seconds. It will then advertise slowly (1600 * 0.625 = 1000 ms) indefinitely. 
``` c
/*! configurable parameters for advertising */
static const appAdvCfg_t datsAdvCfg = {
    { 5000,    0}, /*! Advertising durations in ms, 0 is infinite */
    { 300,  1600}  /*! Advertising intervals in 0.625 ms units */
};
```

### How do I change the connection parameters?

Only the master of the connection can change the connection parameters. Peripheral devices can request a change, but only the master can accept and set the connection parameters. Cell phones and mobile operating systems have different restrictions on connection parameters.

Peripheral applications have the following structure that is used to request connection parameter updates. 

```c
/* iOS connection parameter update requirements

 The connection parameter request may be rejected if it does not meet the following guidelines:
 * Peripheral Latency of up to 30 connection intervals.
 * Supervision Timeout from 2 seconds to 6 seconds.
 * Interval Min of at least 15 ms.
 * Interval Min is a multiple of 15 ms.
 * One of the following:
   * Interval Max at least 15 ms greater than Interval Min.
   * Interval Max and Interval Min both set to 15 ms.
   * Interval Max * (Peripheral Latency + 1) of 2 seconds or less.
   * Supervision Timeout greater than Interval Max * (Peripheral Latency + 1) * 3.
*/

/*! configurable parameters for connection parameter update */
static const appUpdateCfg_t datsUpdateCfg = {
    0,
    /*! ^ Connection idle period in ms before attempting
    connection parameter update. set to zero to disable */
    (15 * 8 / 1.25), /*! Minimum connection interval in 1.25ms units */
    (15 * 12 / 1.25), /*! Maximum connection interval in 1.25ms units */
    0, /*! Connection latency */
    600, /*! Supervision timeout in 10ms units */
    5 /*! Number of update attempts before giving up */
};
```
The DmConnUpdate() function can also be used to request a connection parameter update from the peripheral or initiate one from the master.

```c
/*************************************************************************************************/
/*!
 *  \brief  Update the connection parameters of an open connection
 *
 *  \param  connId      Connection identifier.
 *  \param  pConnSpec   Connection specification.
 *
 *  \return None.
 */
/*************************************************************************************************/
void DmConnUpdate(dmConnId_t connId, hciConnSpec_t *pConnSpec);
```

### How do I use the low-power modes?

All of the applications will enter sleep mode in idle when built with ```DEBUG=0```. The Wireless Stack Framework (WSF) operating system will call PalSysSleep() mode when idle. With ```DEBUG=1```, the CPU will stay in active mode to leave the debugger enabled.

To enter the lowest power states, refer to the BLE_FreeRTOS application. This will create FreeRTOS tasks for the Cordio stack and allow users to add additional tasks. Enable ```configUSE_TICKLESS_IDLE```, and the device will enter standby mode and deep sleep between events.

**WARNING:** The CPU debugger is disabled in sleep modes. If your application enters sleep mode directly after reset, it will be difficult to debug and reprogram.

### How do I send unformatted data like a UART?

Unfortunately, there is no Bluetooth SIG-defined standard for this protocol. This stack has a proprietary data transfer service that is used to transmit unformatted data between devices. Refer to the BLE_dats (BLE Data Server) for the peripheral application. You can connect to this device with the BLE_datc (BLE data client) application to see simple data transmission. Refer to the BLE_dats and BLE_dats [README](../docs/Applications/BLE_datc_dats.md) for more information.

## Additional Documentation

Documentation for Python tools used for Bluetooth development and debugging can be found [here](../../../Tools/Bluetooth/README.md).

Documentation for each of the supporting applications can be found below:

- [BLE4](Applications/BLE4_ctr.md)
- [BLE5](Applications/BLE5_ctr.md)
- [FreeRTOS](Applications/BLE_FreeRTOS.md)
- [DAT Client/Server](Applications/BLE_datc_dats.md)
- [BLE FCC](Applications/BLE_fcc.md)
- [BLE Fit](Applications/BLE_fit.md)
- [BLE MCS](Applications/BLE_mcs.md)
- [OTA Client/Server](Applications/BLE_otac_otas.md)
- [BLE Periph](Applications/BLE_periph.md)
- [RF Test](Applications/RF_Test.md)

## Certification

Bluetooth LE Mesh solution implementing the Bluetooth Mesh Profile 1.0 and the Bluetooth Mesh Model 1.0 wireless technical specifications

* [QDID 116593](https://launchstudio.bluetooth.com/ListingDetails/66212)

Bluetooth LE Host protocol stack implementing Bluetooth Core 5.2 specification

* [QDID 146344](https://launchstudio.bluetooth.com/ListingDetails/103670)

Bluetooth LE Link Layer protocol stack implementing Bluetooth 5.2 specification

* [QDID 146281](https://launchstudio.bluetooth.com/ListingDetails/103599)

MAX32655 controller subsystem

* [QDID 159701](https://launchstudio.bluetooth.com/ListingDetails/119468)

MAX32665 controller subsystem

* [QDID 142345](https://launchstudio.bluetooth.com/ListingDetails/98880)

Consult the [Bluetooth Qualification Process](https://www.bluetooth.com/develop-with-bluetooth/qualification-listing) for further details regarding certification.
