# MAX78000 Snake Game Demo using Real Time Speech Recognition

## Description

This demo is a fun activity to play a snake game using voice commands. It is an extension of keyword spotting demo that demonstrates the use of speech recognition. 
To perform speech recognition, a trained model is used to detect certain keywords in this demo.

The following keywords are used in this game:

 ['**up', 'down', 'left', 'right', 'stop', 'go', 'one', 'two', 'three', 'four**']

Rest of the keywords fall under "**Unknown**" category.

The game has 4 levels and is selected using keywords 1 to 4. 

Here are the instructions to play this game using speech command:  

1. 'one', 'two', 'three' or 'four' command is used to select the game level

2. 'go' command is used to start the game

3. 'up', 'down', 'left' and 'right' command will move the snake in a direction as indicated in name

4. 'stop' command will exit the game

5. The game is over if *stop* command is encountered or if the snake collides with itself

## Software

### Project Usage

Universal instructions on building, flashing, and debugging this project can be found in the **[MSDK User Guide](https://analog-devices-msdk.github.io/msdk/USERGUIDE/)**.

### Project-Specific Build Notes

* This project comes pre-configured for the MAX78000EVKIT.  See [Board Support Packages](https://analog-devices-msdk.github.io/msdk/USERGUIDE/#board-support-packages) in the UG for instructions on changing the target board.

## MAX78000 EVKIT operations

*   Place TFT display on the display header.
*   Connect a USB cable between the PC and the CN1 (USB/PWR) connector.
*   Place jumper P0\_0 and P0\_1 on UART\_0\_EN header JH1.
*   Open a serial port application on the PC and connect to Ev-Kit's console UART at 115200, 8-N-1 configuration.
*   Follow instructions on TFT display

## MAX78000 Feather operations

The TFT display is not supplied with the MAX78000 Feather board.

**To run this demo application you need TFT display.**

The MAX78000 Feather compatible 2.4'' TFT FeatherWing display can be ordered here:

https://learn.adafruit.com/adafruit-2-4-tft-touch-screen-featherwing

This TFT display comes fully assembled with dual sockets for MAX78000 Feather to plug into.
While using TFT display keep its power switch in "ON" position. The TFT "Reset" button also can be used as Feather reset. 

Press PB1 (SW1) button to start demo.

## Expected Output
The instructions about how to set level and start the game are displayed on TFT LCD.  

The Console UART of the device will output these messages:

```
**************** Snake Game ****************

*** Init LCD ***

*** CNN Kernel load ***

*** I2S & Mic Init ***

*** READY ***
025472 Word starts from index: 21504, avg:432 > 350 
033792: Word ends, Appends 4224 zeros 
033792: Starts CNN: 1
033792: Completes CNN: 1
CNN Time: 2577 us
Min: -45,   Max: 51 
----------------------------------------- 
Detected word: FOUR (99.2%)
----------------------------------------- 
105344 Word starts from index: 101376, avg:410 > 350 
110848: Word ends, Appends 7040 zeros 
110848: Starts CNN: 2
110848: Completes CNN: 2
CNN Time: 2577 us
Min: -33,   Max: 40 
----------------------------------------- 
Detected word: GO (100.0%)
----------------------------------------- 
```

## References

https://github.com/MaximIntegratedAI/MaximAI_Documentation