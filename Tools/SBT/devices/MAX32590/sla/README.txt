#
#
#

blinkled_P2_8.sbin : Blinkled example that drive P2.8 pin
blinkled_P2_9.sbin : Blinkled example that drive P2.9 pin

These examples was tested on "EvKIT LAC-2110-D"

Command to sign image:
--------------------------------------
    To build for NAND
    ------------
        build_scp_session -c MAX32590 scp_packages blinkled_P2_8.sbin 
    
    To build for SPI Flash
    --------------
        build_scp_session -c MAX32590-SPINOR scp_packages blinkled_P2_8.sbin 
    
    
    Load it on device: send_scp -c MAX32590 -s COMX scp_packages/
