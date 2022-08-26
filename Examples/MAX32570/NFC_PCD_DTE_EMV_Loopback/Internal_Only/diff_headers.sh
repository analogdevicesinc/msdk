#!/bin/sh

echo \*\* mml_nfc_pcd_rf_driver \*\*
meld ../../../DO_NOT_DISTRIBUTE_Libraries/lib_nfc_pcd_rf_driver_MAX32570/include/nfc/mml_nfc_pcd_rf_driver.h ./include/nfc/mml_nfc_pcd_rf_driver.h

echo \*\* mml_nfc_pcd_port.h \*\*
meld ../../../DO_NOT_DISTRIBUTE_Libraries/lib_nfc_pcd_rf_driver_MAX32570/include/nfc/mml_nfc_pcd_port.h ./include/nfc/mml_nfc_pcd_port.h

echo \*\* pbm_commands.h \*\*
meld ../../../DO_NOT_DISTRIBUTE_Libraries/lib_nfc_pcd_pbm/include/pbm_commands.h ./include/nfc/pbm/pbm_commands.h 
