## Packetcraft Protocol Software

Packetcraft protocol software is a collection of embedded stacks implementing the Bluetooth Low Energy Link Layer, Host, Profile and Mesh specification (www.bluetooth.org).

This repository contains open source release of Packetcraft's software. This is a qualified release and may be used in products. Please consult the [Bluetooth Qualification Process](https://www.bluetooth.com/develop-with-bluetooth/qualification-listing) for further details regarding additional certification requirements.

Release notes
-------------

This latest release of the Packetcraft Host and Packetcraft Controller is Bluetooth 5.2 qualified and implements the following new Bluetooth 5.2 features: LE Isochronous Channels, Enhanced Attribute Protocol, and LE Power Control.

This release includes the following completed requirements for r20.05:

    FW-3340 Isochronous Demo: single BIS data stream
    FW-3354 TCRL.2019-1 compliant
    FW-3359 Core v5.2: LE Isochronous Channels (ISO)
    FW-3360 Core v5.2: LE Power Control
    FW-3361 Core v5.2: Enhanced ATT (EATT)
    FW-3617 Core v5.2: Isochronous Abstraction Layer (ISOAL)
    FW-3726 Core v5.2: Host Support for LE Isochronous Channels (ISO)
    FW-3727 Nordic nRF5 SDK 16.0.0
    FW-3730 Compile BLE host for 64-bit platform
    FW-3736 Light CTL Model
    FW-3738 Mesh v1.0.1 compliant
    FW-3739 TCRL.2019-2 qualification
    FW-3750 Laird BL654 platform
    FW-3767 SBC codec
    FW-3803 GCC compiler support for gcc-arm-none-eabi-9-2019-q4-major
    FW-3820 Protect against SweynTooth vulnerability


Certification
-------------

Bluetooth LE Mesh solution implementing of the Bluetooth Mesh Profile 1.0 and the Bluetooth Mesh Model 1.0 wireless technical specifications

* [QDID 116593](https://launchstudio.bluetooth.com/ListingDetails/66212)

Bluetooth LE Host protocol stack implementing Bluetooth Core 5.2 specification

* [QDID 146344](https://launchstudio.bluetooth.com/ListingDetails/103670)

Bluetooth LE Link Layer protocol stack implementing Bluetooth 5.2 specification

* [QDID 146281](https://launchstudio.bluetooth.com/ListingDetails/103599)


Verification
------------

Packetcraft Mesh is verified with the TCRL.2019-2 compliance tester using the following:

* Bluetooth Profile Tuning Suites 7.6.1

Packetcraft Host is verified with the TCRL.2019-2 compliance tester using the following:

* Bluetooth Profile Tuning Suites 7.6.1

Packetcraft Profiles is verified with the TCRL.2018-2 compliance tester using the following:

* Bluetooth Profile Tuning Suites 7.3.0

Packetcraft Link Layer conforms to the Bluetooth TCRL.2019-2 requirements verified with the following:

* Teledyne Harmony LE Tester version 19.12.16916.21195

This product was compiled and tested with the following version of GNU GCC

* gcc-arm-none-eabi-9-2019-q4-major
