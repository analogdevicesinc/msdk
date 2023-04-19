# Bluetooth Tools
These python tools are used for Bluetooth development and debugging.

## BLE_hci
Host Controller Interface for controlling Bluetooth Controller devices. Uses a serial port and the HCI interface to send commands and receive events.

```
usage: BLE_hci.py [-h] [--monPort [MONPORT]] [--serialPort [SERIALPORT]] [--baud [BAUD]] [-c COMMAND] [serial_port] [baud]

    Bluetooth Low Energy HCI tool.

    This tool is used in tandem with the BLE controller examples. This tools sends
    HCI commands through the serial port to the target device. It will receive and print
    the HCI events received from the target device.

    Serial port is configured as 8N1, no flow control, default baud rate of 115200.
    

positional arguments:
  serial_port           Serial port path or COM#, default: /dev/ttyUSB0
  baud                  Serial port baud rate, default: 115200

options:
  -h, --help            show this help message and exit
  --monPort [MONPORT]   Monitor Trace Msg Serial Port path or COM#, default: 
  --serialPort [SERIALPORT]
                        Serial port path or COM#, default: /dev/ttyUSB0
  --baud [BAUD]         Serial port baud rate, default: 115200
  -c COMMAND, --command COMMAND
                        Commands to run

commands:
    addr                Set the device address
    adv                 Send the advertising commands
    scan                Send the scanning commands and print scan reports. ctrl-c to exit
    init                Send the initiating commands to open a connection
    dataLen             Set the max data length
    sendAcl             Send ACL packets
    sinkAcl             Sink ACL packets, do not send events to host
    connStats           Get the connection stats
    phy                 Update the PHY in the active connection
    reset               Sends a HCI reset command
    listen              Listen for HCI events, print to screen
    txTest (tx)         Execute the transmitter test
    txTestVS (tx)       Execute the transmitter test
    rxTest (rx)         Execute the receiver test
    endTest (end)       End the TX/RX test, print the number of correctly received packets
    txPower (txp)       Set the TX power
    discon (dc)         Send the command to disconnect
    setChMap            Set the connection channel map to a given channel.
    cmd                 Send raw HCI commands
    readReg             Read register, device performs a memcpy from address and returns the value
    writeReg            Write register, device performs a memcpy to memory address
    exit (quit)         Exit the program
    help (h)            Show help message
```

## Mini RCDAT USB
Control a Minicircuits attenuator over USB.

## RS FSL
Control a Rhode and Schwartz FSL Spectrum analyzer over ethernet.
