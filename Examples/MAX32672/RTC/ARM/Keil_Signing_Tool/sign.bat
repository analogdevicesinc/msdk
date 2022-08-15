::@echo off

echo on

rem Paths and filenames needed to compute signature
set TOOL_DIR=.
set OUT_FILE=RTC.axf
set KEY_FILE=.\maximtestcrk.key
set DEBUG_FILE_LAST=DEBUG_LAST
set DEBUG_FILE_BEG=DEBUG_BEG

::rem Create a binary file
echo ------------------------------- STEP 1 ---------------------------------------
%2ARM\ARMCC\bin\fromelf --bincombined --output=%1Objects\%OUT_FILE%.bin %1Objects\%OUT_FILE%

%2ARM\ARMCC\bin\fromelf --bincombined --output=%1Objects\%DEBUG_FILE_BEG%.bin %1Objects\%OUT_FILE%

::Update sla header of image with correct size, output header and app after removing dummy signature(.nosig .sla and .hdr are all binary files)
echo ------------------------------- STEP 2 ---------------------------------------
%3Signature_Tools\sla_tool_v2 %1Objects\%OUT_FILE%.bin %1Objects\%OUT_FILE%.nosig %1Objects\%OUT_FILE%.sla %1Objects\%OUT_FILE%.hdr %1Objects\%OUT_FILE%.deadcode

::Generate the signature using the *.nosig that creates *.sbin, then a diff between *.nosig and *.sbin creates the *.sig
echo ------------------------------- STEP 3 ---------------------------------------
%3Signature_Tools\sign_app -c MAX32672 key_file=%3Signature_Tools\%KEY_FILE% algo=ecdsa header=no ca=%1Objects\%OUT_FILE%.nosig sca=%1Objects\%OUT_FILE%.sbin


::Add 0xdeadcode in the beginning to the signature
echo ------------------------------- STEP 4 ---------------------------------------
copy /b %1Objects\%OUT_FILE%.deadcode + %1Objects\%OUT_FILE%.sig %1Objects\%OUT_FILE%.deadcodesig


::Update the original image with new header (new header contains correct size in the desginated location)
echo ------------------------------- STEP 5 ---------------------------------------
%3Signature_Tools\arm-none-eabi-objcopy -I elf32-littlearm %1Objects\%OUT_FILE% --update-section HEADER=%1Objects\%OUT_FILE%.hdr



::Update the original image (that has the updated *.hdr from STEP 5) with 0xdeadcode + signature
echo ------------------------------- STEP 6 ---------------------------------------
%3Signature_Tools\arm-none-eabi-objcopy -I elf32-littlearm %1Objects\%OUT_FILE% --update-section SB_SLA=%1Objects\%OUT_FILE%.deadcodesig


%2ARM\ARMCC\bin\fromelf --bincombined --output=%1Objects\%DEBUG_FILE_LAST%.bin %1Objects\%OUT_FILE%