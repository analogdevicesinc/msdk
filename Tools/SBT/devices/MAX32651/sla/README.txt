#
#
#

blinkled example is tested on MAX32651 EvKIT Rev F it drives led which is conneted to P2.25 and print debug messages over UART0.

blinkled.bin    -->  includes SLA (Second Level Application) header.
blinkled.sbin   -->  is signed version of .bin file.


Command to sign image:
--------------------------------------
Sign             : sign_app.exe -c MAX32651 blinkled.bin
Build scp session: build_scp_session -c MAX32651 scp_packages blinkled.sbin 
Load it on device: send_scp -c MAX32651 -s COMX scp_packages/

