## Description

An example demonstrating how to use the IPO Calibration feature to calibrate the Internal Primary Oscillator (IPO) using the External Real-Time Clock Oscillator (ERTCO) as the reference.

## Software

### Project Usage

Universal instructions on building, flashing, and debugging this project can be found in the **[MSDK User Guide](https://analogdevicesinc.github.io/msdk/USERGUIDE/)**.

### Project-Specific Build Notes

* This project comes pre-configured for the MAX32657EVKIT.  See [Board Support Packages](https://analogdevicesinc.github.io/msdk/USERGUIDE/#board-support-packages) in the MSDK User Guide for instructions on changing the target board.

## Required Connections

A 32.768Hz crystal needs to be attached to the board for this example to work.

## Expected Output

After the IPO Calibration function is called, both LED0 (ERTCO Clock) and P0.1 (IPO_APB Clock) output the same frequency of 2 Hz.