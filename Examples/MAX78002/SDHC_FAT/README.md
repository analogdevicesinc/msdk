## Description

This example demonstrates the SDHC FAT Filesystem. The terminal prompts with a list of user-selectable tasks to run on the inserted Micro SD Card.

## Software

### Project Usage

Universal instructions on building, flashing, and debugging this project can be found in the [MSDK User Guide](https://analog-devices-msdk.github.io/msdk/USERGUIDE/).

- For a "quick-start" or for first-time users see ["Getting Started"](https://analog-devices-msdk.github.io/msdk/USERGUIDE/#getting-started)
- See ["Development Guide"](https://analog-devices-msdk.github.io/msdk/USERGUIDE/#development-guide) for a detailed reference.

### Project-Specific Build Notes

(None)

## Required Connections

-   Connect a USB cable between the PC and the CN2 (USB/PWR) connector.
-   Connect the 5V power cable at (5V IN).
-   Close jumper (RX - P0.0) and (TX - P0.1) at Headers JP23 (UART 0 EN).
-   Open an terminal application on the PC and connect to the EV kit's console UART at 115200, 8-N-1.

## Expected Output

```
***** MAX78002 SDHC FAT Filesystem Example *****
Card inserted.
Card Initialized.
Card type: SDHC
SD clock ratio (at card) 2:1

Choose one of the following options:
0. Find the Size of the SD Card and Free Space
1. Format the Card
2. Manually Mount Card
3. List Contents of Current Directory
4. Create a Directory
5. Move into a Directory (cd)
6. Create a File of Random Data
7. Add Random Data to an Existing File
8. Delete a File
9. Format Card and Run Exmaple of FatFS Operations
10. Unmount Card and Quit

```

