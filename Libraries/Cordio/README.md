# Cordio
This library implements the Bluetooth Low Energy stack , forked from [Packetcraft](https://github.com/packetcraft-inc/stacks). It depends on additional libraries within the msdk repository. All of the applications utilizing this library can be found in the [Examples](../../Examples) directory.

## Getting Started
The best application to get started with is the ```BLE_periph``` application. It is a simple periphal application that will allow you to advertise and connect with a central device such as a smart phone. 

Follow the MSDK [README](../../README.md) instructions to install the necessary tools and create a new project. Build and run the ```BLE_periph``` application for the appropriate target. Refer to the applicaiton README for additional details.

The Bluetooth Special Interest Group (SIG) has documentation available on their [website](https://www.bluetooth.com/). Refer to the [documentation](https://www.bluetooth.com/specifications/specs/) page to see the latest core specification. This will be your best resource for learning more about Bluetooth Low Energy.

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

## Additional Documentation
The Cordio architecture is described [here](docs/ARCHITECTURE.md). 

Documentation for each of the supporting applications can be found [here](docs/Applications).

Documentation for Python tools used for Bluetooth development and debugging can be found [here](../../Tools/Bluetooth/README.md).

