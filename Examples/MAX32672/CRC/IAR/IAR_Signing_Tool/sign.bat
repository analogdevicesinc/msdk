::@echo off

echo on

rem Paths and filenames needed to compute signature
set TOOL_DIR=.
set OUT_FILE=CRC.out
set KEY_FILE=.\maximtestcrk.key
set EXE_LOC=Rev_A
rem set EXE_LOC=Debug 

::rem Get binary image
echo ----------------- STEP 1 ------------------------
%2Signature_Tools\arm-none-eabi-objcopy -I elf32-littlearm %1\%EXE_LOC%\Exe\%OUT_FILE% -O binary %1\%EXE_LOC%\Exe\%OUT_FILE%.bin

echo ----------------- STEP 2 ------------------------
::Update sla header of image with correct size, output header and app with dummy signature removed (.nosig .sla and .hdr are all binary files)
%2Signature_Tools\sla_tool %1\%EXE_LOC%\Exe\%OUT_FILE%.bin %1\%EXE_LOC%\Exe\%OUT_FILE%.nosig %1\%EXE_LOC%\Exe\%OUT_FILE%.sla %1\%EXE_LOC%\Exe\%OUT_FILE%.hdr

echo ----------------- STEP 3 ------------------------
::Generate the signature
%2Signature_Tools\sign_app -c MAX32672 key_file=%2Signature_Tools\%KEY_FILE% algo=ecdsa header=no ca=%1\%EXE_LOC%\Exe\%OUT_FILE%.nosig sca=%1\%EXE_LOC%\Exe\%OUT_FILE%.sbin

echo ----------------- STEP 4 ------------------------
::Append signature onto end of app
copy /b %1\%EXE_LOC%\Exe\%OUT_FILE%.sla + %1\%EXE_LOC%\Exe\%OUT_FILE%.sig %1\%EXE_LOC%\Exe\%OUT_FILE%.sla

echo ----------------- STEP 5 ------------------------
::Update the original image with new header (new header contains correct size)
%2Signature_Tools\arm-none-eabi-objcopy -I elf32-littlearm %1\%EXE_LOC%\Exe\%OUT_FILE% --update-section HEADER=%1\%EXE_LOC%\Exe\%OUT_FILE%.hdr

echo ----------------- STEP 6 ------------------------
::Update the original image with new app + signature
%2Signature_Tools\arm-none-eabi-objcopy -I elf32-littlearm %1\%EXE_LOC%\Exe\%OUT_FILE% --update-section CODE=%1\%EXE_LOC%\Exe\%OUT_FILE%.sla