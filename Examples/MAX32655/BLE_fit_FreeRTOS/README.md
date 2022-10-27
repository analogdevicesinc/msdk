# Description

Bluetooth fitness device. Showcases heart rate, battery level, running speed and cadence.

Please refer to the [Examples/MAX32655/BLE_fit/README.md](../BLE_fit/README.md) for details.

This demo project modifies the [Examples/MAX32655/BLE_fit/](../BLE_fit/) project and implements the application in FreeRTOS.

In the serial console, two new commands "cmd ps s" and "cmd ps l" are added to display the FreeRTOS task related information.
```
cmd ps s  
Task            Run time cnt    Run time percentage  
***************************************************  
CordioH        	73		<1  
IDLE           	13641		99  
Tmr Svc        	0		<1  
CordioM        	0		<1  
```

```
cmd ps l  
Task          State  Priority  Stack    #  
**************************************  
CordioH        	X	2	3654	2  
IDLE           	R	0	117	3  
Tmr Svc        	B	2	90	4  
CordioM        	S	3	3982	1  
```