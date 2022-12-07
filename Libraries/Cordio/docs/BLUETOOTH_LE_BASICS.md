# Bluetooth LE Basics

## References
The Bluetooth Special Interest Group (SIG) has documentation available on their [website](https://www.bluetooth.com/). Refer to the [documentation](https://www.bluetooth.com/specifications/specs/) page to see the latest core specification. This will be your best resource for learning the details of Bluetooth Low Energy.

These books are also excellent references for developers.

* *Getting Started with Bluetooth Low Energy* by O'Reilly
* *Bluetooth Low Energy: The Developer's Handbook* by Robin Heydon

## Architecture
The Bluetooth stack closely resembles the layers of the network stack. We have the application layer at the top and the physical layer at the bottom. Each layer encapsulates the data and passes it to the appropriate section of the upper and lower layers.

<p align="center">
  <img width="500" src="./pics/Stack.PNG">
</p>

The Host Conroller Interface (HCI) is the common point where devices are split. Typically this this interface is over an asynchronous protocol such as UART. Some devices will define proprietary interfaces between the application and host layers. Multi-core SOCs can also use the HCI or proprietay interfaces to split the stack between multiple CPUs.


When testing the Controller layers, test equipment will have a USB interface and act as a Host device. Devices under test will use a USB to UART adapter and act as Controller devices. 

<p align="center">
  <img width="500" src="./pics/HCI.PNG">
</p>

## States
These are the common states used in Bluetooth LE communication. Typically devices will be in only one of these states at a time, but it is possible for devices to be in all simultaneously. 

### Advertising

### Scanning / Initiating

### Connected