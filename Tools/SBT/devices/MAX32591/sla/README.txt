#
#
#

blinkled_P3_29.sbin : Blinkled example that drive P3.29 pin
blinkled_P3_30.sbin : Blinkled example that drive P3.30 pin

These examples was tested on "MAX32591 EvKIT REV A"

Command to sign image:
--------------------------------------
    To build for NAND
    ------------
        build_scp_session -c MAX32591 scp_packages blinkled_P3_29.sbin 
    
    To build for SPI Flash
    --------------
        build_scp_session -c MAX32591-SPINOR scp_packages blinkled_P3_29.sbin 
    
    
    Load it on device: send_scp -c MAX32591 -s COMX scp_packages/
