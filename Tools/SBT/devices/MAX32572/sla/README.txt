blinkled_P1_30_EvKit_RevA.sbin : Signed version of blinkled_P1_30_EvKit_RevA.bin, This example drive P1.30 pin

These example is tested on "MAX32572 EvKIT Rev A"

Commands:
--------------------------------------
To Sign : sign_app -c MAX32572 blinkled_P1_30_EvKit_RevA.bin
To Build: build_scp_session -c MAX32572 scp_packets blinkled_P1_30_EvKit_RevA.sbin
To Load : send_scp -c MAX32572 -s COMX scp_packets/

Alternatively, SBT GUI can be used to create and send SBT packets
