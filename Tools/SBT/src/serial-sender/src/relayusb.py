#!/usr/bin/env python2

import sys
import usb.core
import usb.util
import time
import serial


class RelayCtrl:
    def __init__(self):
        # find our device
        self.dev = usb.core.find(idVendor=0x0403, idProduct=0x6001)

        # was it found?
        if self.dev is None:
            raise ValueError('Device not found')

        # set the active configuration. With no arguments, the first
        # configuration will be the active one
        self.dev.set_configuration()

        # get an endpoint instance
        cfg = self.dev.get_active_configuration()
        intf = cfg[(0, 0)]

        self.ep = usb.util.find_descriptor(
            intf,
            # match the first OUT endpoint
            custom_match= \
                lambda e: \
                    usb.util.endpoint_direction(e.bEndpointAddress) == \
                    usb.util.ENDPOINT_OUT)

        assert self.ep is not None

        # write the data
        self.dev.ctrl_transfer(0x40, 0, 0, 0, 0)
        self.dev.ctrl_transfer(0xC0, 5, 0, 0, 2)
        self.dev.ctrl_transfer(0x40, 3, 0x8003, 0, 0)
        self.dev.ctrl_transfer(0x40, 11, 0x04ff, 0, 0)
        self.dev.ctrl_transfer(0xC0, 12, 0, 0, 1)
        self.dev.ctrl_transfer(0xC0, 12, 0, 0, 1)
        self.dev.ctrl_transfer(0xC0, 12, 0, 0, 1)
        self.dev.ctrl_transfer(0xC0, 12, 0, 0, 1)

    # ev.ctrl_transfer(0xC0, 12, 0, 0, 1)

    def on(self):
        self.ep.write('\x03')
        self.ep.write('\x03')
        self.dev.ctrl_transfer(0xC0, 12, 0, 0, 1)
        self.dev.ctrl_transfer(0xC0, 12, 0, 0, 1)

    def off(self):
        self.ep.write('\x00')
        self.ep.write('\x00')
        self.dev.ctrl_transfer(0xC0, 12, 0, 0, 1)
        self.dev.ctrl_transfer(0xC0, 12, 0, 0, 1)

    def reset(self):
        self.dev.reset()
        self.dev.ctrl_transfer(0x40, 0, 0, 0, 0)
        self.dev.ctrl_transfer(0xC0, 5, 0, 0, 2)
        self.dev.ctrl_transfer(0x40, 3, 0x8003, 0, 0)
        self.dev.ctrl_transfer(0x40, 11, 0x04ff, 0, 0)
        self.dev.ctrl_transfer(0xC0, 12, 0, 0, 1)
        self.dev.ctrl_transfer(0xC0, 12, 0, 0, 1)
        self.dev.ctrl_transfer(0xC0, 12, 0, 0, 1)
        self.dev.ctrl_transfer(0xC0, 12, 0, 0, 1)

    def release(self):
        usb.util.dispose_resources(self.dev)