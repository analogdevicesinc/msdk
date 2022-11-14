define olimex
set $OLIMEX = 1
set $SEGGER = 0
target remote :3333
end

define mrh
monitor reset halt
end

define do
dont-repeat
make
reload
end

define go
dont-repeat
reload
end

define reload
dont-repeat
mrh
load
c
end

# Generic
define jtag_khz 
dont-repeat
monitor adapter_khz $arg0
end

define cpu_info
monitor arm926ejs cache_info
end

define restart
mrh
set $pc=0x10000020
c
end

olimex
#mrh

# Fix Windows gdb issue, by sending stop to our process
echo \n
echo **> To make windows gdb behave like Linux
echo where Cntrl-C pauses, run this command in seperate window \n
echo kill -STOP `ps | grep gdb | awk '{print $1}'`
echo \n
