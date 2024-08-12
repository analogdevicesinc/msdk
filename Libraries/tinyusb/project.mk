include tinyusb.mk

override PROJECT := tinyusb

LIB_PERIPHDRIVERS := 1

# C source files
SRCS += \
        src/tusb.c \
        src/common/tusb_fifo.c \
        src/device/usbd.c \
        src/device/usbd_control.c \
        src/class/audio/audio_device.c \
        src/class/bth/bth_device.c \
        src/class/cdc/cdc_device.c \
        src/class/dfu/dfu_device.c \
        src/class/dfu/dfu_rt_device.c \
        src/class/hid/hid_device.c \
        src/class/midi/midi_device.c \
        src/class/msc/msc_device.c \
        src/class/net/ecm_rndis_device.c \
        src/class/net/ncm_device.c \
        src/class/usbtmc/usbtmc_device.c \
        src/class/video/video_device.c \
        src/class/vendor/vendor_device.c \
        src/class/cdc/cdc_host.c \
        src/class/hid/hid_host.c \
        src/class/msc/msc_host.c \
        src/class/vendor/vendor_host.c \
        src/portable/mentor/musb/dcd_musb.c \
        hw/bsp/board.c

override .DEFAULT_GOAL := lib
