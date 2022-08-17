#
#
#

blinkled example is tested on MAX32665/66/67/68 EvKit Rev C it drives led which is conneted to P1.14 and print debug messages over UART1.

blinkled.bin    -->  does not include SLA (Second Level Application) header.
blinkled.sbin   -->  is signed version of .bin file. The SLA header is genereted by sign_app by below command


Command to sign image:
--------------------------------------
sign_app.exe -c MAX32666 header=yes blinkled.bin


Other configuration is default one which comes from device.ini as below:

[MAX32666]
header=yes
key_file=%MAXIM_SBT_DIR%\devices\MAX32666\keys\maximtestcrk.key
rom_version=01000000
application_version=01000000
load_address=10000000
jump_address=1000054C
