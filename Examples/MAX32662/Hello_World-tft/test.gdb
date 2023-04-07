# Create some variables for easy access to flc registers.
set variable $fctl_base_addr=0x40029000
set variable $fctl_faddr=$fctl_base_addr+0x00
set variable $fctl_fckdiv=$fctl_base_addr+0x04
set variable $fctl_fcntl=$fctl_base_addr+0x08
set variable $fctl_fdata0=$fctl_base_addr+0x30
set variable $fctl_fdata1=$fctl_base_addr+0x34
set variable $fctl_fdata2=$fctl_base_addr+0x38
set variable $fctl_fdata3=$fctl_base_addr+0x3C
set variable $fctl_acntl=$fctl_base_addr+0x40



# Create a function to write the flash using direct register writes.
# arg0 = address
# arg1 = 32-bit data
# arg2 = 32-bit data
# arg3 = 32-bit data
# arg4 = 32-bit data
define write_flash_128
# Set flash clock divide to 120
set *$fctl_fckdiv=120
# Unlock and Set width to 128 (bit 4 to zero)
set *$fctl_fcntl=*$fctl_fcntl | 0x20000000
set *$fctl_fcntl=*$fctl_fcntl & ~0x00000010
# Set address
set *$fctl_faddr=$arg0
# Set 128-bits of data
set *$fctl_fdata0=$arg1
set *$fctl_fdata1=$arg2
set *$fctl_fdata2=$arg3
set *$fctl_fdata3=$arg4
# start flash operation
set *$fctl_fcntl=*$fctl_fcntl | 0x20000001
while (0 != (*$fctl_fcntl & 0x7))
# Wait for flash write to complete
end
end