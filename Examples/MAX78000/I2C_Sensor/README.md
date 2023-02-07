## Description

This example demonstrates how to take temperature readings from the MAX31889 using the I2C.

After initialization, a new reading is printed to the terminal every second.

If using the Standard EvKit (EvKit_V1):
-   Connect a USB cable between the PC and the CN1 (USB/PWR) connector.
-   Connect pins 1 and 2 (P0_1) of the JH1 (UART 0 EN) header.
-   Open a terminal application on the PC and connect to the EV kit's console UART at 115200, 8-N-1.
-   Close jumper JP1 (LED1 EN).
-   Close jumper JP2 (LED2 EN).
-	Select "EvKit\_V1" for _BOARD_ in "project.mk"
-   You must connect pin 8 on camera header J4 (SCL), pin 6 on camera header J4 (SDA), VDD and GND to corresponding pins of MAX31889 EVKIT_A board (via J3 terminal)

If using the MAX78000FTHR (FTHR_RevA)
-   Connect a USB cable between the PC and the CN1 (USB/PWR) connector.
-	Open a terminal application on the PC and connect to the EV kit's console UART at 115200, 8-N-1.
-	Select "FTHR\_RevA" for _BOARD_ in "project.mk"
-   You must connect pin 11 on header J4 (SCL), pin 12 on header J4 (SDA), VDD and GND to corresponding pins of MAX31889 EVKIT_A board (via J3 terminal)

## Expected Output

```
****************** I2C SENSOR DEMO *******************

-->Temperature: 21.920000 °C
-->Temperature: 21.910000 °C
-->Temperature: 21.895000 °C
-->Temperature: 21.924999 °C
-->Temperature: 21.910000 °C
-->Temperature: 21.895000 °C
-->Temperature: 21.889999 °C
-->Temperature: 21.859999 °C
-->Temperature: 21.885000 °C
-->Temperature: 21.904999 °C
-->Temperature: 21.889999 °C
-->Temperature: 21.895000 °C

...
```

## Instructions for Using the MAX31889 Driver

### Opening MAX31889 Sensor Driver
```c
	mxc_i2c_sensor_driver_t MAX31889 = MAX31889_Open();
```



### Initializing the MAX31889
```c
	MAX31889.init(I2C_MASTER, MAX31889_I2C_SLAVE_ADDR0);
```
NOTE: This functions inializes the I2C peripheral used to communicate with the MAX31889, the I2C pins, and the MAX31889 itself.



### Taking a Temperature Reading
```c
	error = MAX31889.read(&temperature);
```


