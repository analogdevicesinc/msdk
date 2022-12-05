# Cordio
This library implements the Bluetooth Low Energy stack , forked from [Packetcraft](https://github.com/packetcraft-inc/stacks). It depends on additional libraries within the msdk repository. All of the applications utilizing this library can be found in the [Examples](../../Examples) directory.

## Getting Started
The best application to get started with is the [BLE_periph](docs/Applications/BLE_periph.md) application. It is a simple periphal application that will allow you to advertise and connect with a central device such as a smart phone. 

Follow the MSDK [README](../../README.md) instructions to install the necessary tools and create a new project. Build and run the BLE_periph application for the appropriate target.

## ADI Attach
ADI Attach is a smart phone applicaiton that can be used for Bluetooth debugging and development.
* Scan for advertising peripherals.
* Connect to devices and discover profiles, services.
* Read and write charcteristics.
* Subscribe to noficiations.
* Perform over-the-air firmware updates with supporing devices. 

## BLE-PyDex
BLE-PyDex is a hardware agnostic Bluetooth device explorer designed to aid in the development and debugging of Bluetooth applications.

https://github.com/EdwinFairchild/BLE-PyDex

## Certification

Bluetooth LE Mesh solution implementing of the Bluetooth Mesh Profile 1.0 and the Bluetooth Mesh Model 1.0 wireless technical specifications

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

## Supported Features

* **LE Dual Mode Topology:** Advertise/Scan while maintaining multiple adjacent connections.
* **2M PHY:** High speed PHY with 2 Mbps symbol rate.
* **Coded PHY:** Forward error correction, 125 kbps or 500 kbps symbol rate.
* **Extended Advertising:** Enhanced advertising and scanning.
* **Periodic Advertising:** Broadcasting and receiving periodic advertisements.
* **EATT:** Enhanced Attribute protocol.
* **LE Secure Connections:** LE Secure Connections is an enhanced security feature introduced in Bluetooth v4.2. It uses a Federal Information Processing Standards (FIPS) compliant algorithm called Elliptic Curve Diffie Hellman (ECDH) for key generation.
* **Data Length Extension:** Extend the maximum data length supported in a connection.
* **LE Power Control:** Dynamic TX power control.

## Features in development
* **Direction finding:** Detect the direction of the incoming signal. Also known as Angle of Arrival (AoA) and Angle of Departure (AoD). Hardware support on MAX32655 and MAX32690, unavailable on MAX32665, software in development. 
* **LE Audio:** Isochronous audio with Bluetooth LE. Hardare support on all platforms, software in development. 

## Frequently asked questions

### How do I change the advertising parameters?
Peripheral applications will have a static structure that contains all of the advertising parameters. If run-time changes are desired, you must call ```AppAdvStop()``` before changing the parameters and ```AppAdvStart()``` to resume.


With this configuration, the device will advertise at a fast interval (300 * 0.625 = 187.5 ms) for 5 seconds. It will then advertise slowly (1600 * 0.625 = 1000 ms) indefinetly. 
``` c
/*! configurable parameters for advertising */
static const appAdvCfg_t datsAdvCfg = {
    { 5000,    0}, /*! Advertising durations in ms, 0 is infinite */
    { 300,  1600}  /*! Advertising intervals in 0.625 ms units */
};
```

### How do I change the connection parameters?
Only the master of the connection can change the connection parameters. Peripheral devices can request a change, but only the master can accept and set the connection parameters. Cell phones and mobile operatings systems have different restrictions on the connection parameters.

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

### How do I use the low power modes?
All of the applications will enter sleep mode in idle when build with ```DEBUG=0```. The Wireless Stack Framework (WSF) operating system will call PalSysSleep() mode when idle. With ```DEBUG=1```, the CPU will stay in active mode to leave the debugger enabled.

To enter the lowest power states, refer to the BLE_FreeRTOS application. This will create FreeRTOS tasks for the Cordio stack and allow users to add additional tasks. Enable ```configUSE_TICKLESS_IDLE``` and the device will enter standby mode and deep sleep between events.

**WARNING:** The CPU debugger is disabled in sleep modes. If your application enters sleep mode directly after reset, it will be difficult to debug and reporgram.

### How do I send unformatted data like a UART?
Unfortunatly there is not a Bluetooth SIG defined standard for this protocol. This stack has a proprietary data transfer service that is used to transmit unformatted data between devices. Refer to the BLE_dats (BLE Data Server) for the peripheral application. You can connect to this device with the BLE_datc (BLE data client) application to see simple data transmission. Refer to the BLE_dats and BLE_dats [README](docs/Applications/BLE_datc_dats.md) for more information.

## Additional Documentation
The Cordio architecture is described [here](docs/ARCHITECTURE.md). 

Documentation for each of the supporting applications can be found [here](docs/Applications).

Documentation for Python tools used for Bluetooth development and debugging can be found [here](../../Tools/Bluetooth/README.md).
