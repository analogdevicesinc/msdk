## README notes for SVD Scripts

### SVD

The SVD Scripts create the register files and part SVD file by referencing the individual peripheral SVD files within the main msdk repo. Each IP Core revision supports a corresponding peripheral SVD file which contains all related register information for a specific peripheral.

### Requirements running in Linux

The SVD scripts require an xml-formatter when generating the part SVD file.

`sudo apt install xmlformat-perl`

Sometimes, the **makeRegs.sh** files are formatted in DOS and need to be re-formatted to UNIX before you can successfully run the SVD scripts. Simply run these commands if you come across format errors:
`sudo apt install dos2unix`
`dos2unix makeRegs.sh`

### How to run svd_add_peripheral_modified.py in shell scripts

The **svd_add_peripheral_modified.py** script consolidates all the peripheral SVD files, that a device supports, into a generated part SVD file (e.g. max32572.svd). All supported peripheral SVD files are listed in **chip_periph.txt** for each MCU in **SVD/Devices/**.

Example: makeRegs.sh for ME55

`python3 ../../svd_add_peripheral_modified.py chip_periph.txt chip_test/max32572.svd [MAXIM_PATH]`

### Notes for Shell Scripts

The optional `-r` parameter is used to generate the reva register files (private).

Example: makeRegs.sh for ME55

`python3 ../../svd_add_peripheral_modified.py chip_periph.txt chip_test/max32572.svd [MAXIM_PATH] -r`

### Notes for Devices/Private Directory

This directory is used to generate all the "private" \_REVA\_ register files.

### svd_removal.py

This file was created to remove sections of a generated register file or SVD file.

This is useful for things like the AFE registers in the ME16. The SVD scripts first consolidates all the peripheral SVD files into a single part SVD file (<part_number>.svd - max32675.svd). Due to the ME16 architecture, the AFE registers shouldn't be included in the max32675.svd file since those registers are internally accessed via UART. However, the AFE register files should still be generated for easier use.

The register struct is also not useful in the AFE register files because these registers aren't located in the ARM memory map, and the range for the AFE register offsets is large and mainly causes bloat in the register file.

With that said, the svd_removal.py app will remove the AFE registers from max32675.svd and the register structs from the AFE register files after all the files are generated.

In the future, if other sections of the SVD or register files should be selectively deleted, you could easily add more options to remove other sections not currently supported in svd_removal.py.

### SVD Naming Notes

There's some confusion for the core system register names. If you reference the Cortex-M3/M4 Family Resource Document or a parts' design register document (e.g. ME16_Registers.pdf), the system intialization register names from these documents can get confusing. There are "Battery backed" and "Non-Battery Backed" System register names that doesn't intuitively translate to the names used in the SDK. Below is a cheatsheet for figuring out what names equate to what whenever you go through the documents and the SVD files during bring-up.

TRIMSIR - 0x40005400 - Battery Backed SIR / TRIM Registers - BBSIR
TRIMSIR - ECC - Same locations, but please use TRIMSIR over ECC.
SIR - 0x40000400 - Non-Battery Backed SIR - SIR
BBFC - Battery Backed Function Control Registers

SFCC - Code Cache
SRCC - Data Cache

