#!/bin/bash
# MakeResults=$(make BOARD="FTHR_RevA"  2>&1> /dev/null)
# if [ "$MakeResults" != "" ]; then
# echo "++++ ERROR ++++";
# echo "$MakeResults";
# exit 1;
# else
# echo "Build Success!";
# echo "Loading code ...";
openocd -s $MAXIM_PATH/Tools/OpenOCD/scripts -f interface/cmsis-dap.cfg -f target/max78000.cfg -c "program build/MAX78000.elf reset exit";
#fi

