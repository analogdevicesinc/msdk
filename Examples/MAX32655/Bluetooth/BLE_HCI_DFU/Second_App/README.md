# Description

This is an example to create second application. It should use the same linker script in `BLT` folder to make the memory layout consistent. 

The command I used to dump the binary from GDB:
```
dump binary memory hello_world.bin 0x10004000 0x10040000
```

0x10004000 is the starting address you want to dump.
0x10040000 is the ending address of your program to be dumped.