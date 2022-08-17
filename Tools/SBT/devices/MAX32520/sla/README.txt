#
#
#

blinkled_P1_6.sbin : Signed version of blinkled_P1_6.bin, This example drive P1.6 pin
blinkled_P1_7.sbin : Signed version of blinkled_P1_7.bin, This example drive P1.7 pin

These examples was tested on "MAX32520 EvKIT Rev B"

Commands:
--------------------------------------
To Sign : sign_app -c MAX32520 blinkled_P1_6.bin
To Build: build_scp_session -c MAX32520 scp_packages blinkled_P1_6.sbin
To Load : send_scp -c MAX32520 -s COMX scp_packages/
