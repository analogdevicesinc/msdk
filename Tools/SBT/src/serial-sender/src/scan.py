#!/usr/bin/env python
# -*- coding: UTF-8 -*-
#
# src/scan.py
#
# ----------------------------------------------------------------------------
# Copyright © 2009-2018, Maxim Integrated Products
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#     * Redistributions of source code must retain the above copyright
#       notice, this list of conditions and the following disclaimer.
#     * Redistributions in binary form must reproduce the above copyright
#       notice, this list of conditions and the following disclaimer in the
#       documentation and/or other materials provided with the distribution.
#     * Neither the name of the Maxim Integrated Products nor the
#       names of its contributors may be used to endorse or promote products
#       derived from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY MAXIM INTEGRATED PRODUCTS ''AS IS'' AND ANY
# EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
# WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
# DISCLAIMED. IN NO EVENT SHALL MAXIM INTEGRATED PRODUCTS BE LIABLE FOR ANY
# DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
# (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
# ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
# (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
# SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
# ----------------------------------------------------------------------------
#
# Created on: Dec 16, 2009
# Author: Grégory Romé <gregory.rome@maxim-ic.com>
# Author: Benjamin VINOT <benjamin.vinot@maximintegrated.com>
#

""" Serial port Scanner

:Summary: Serial port Scanner
:Author: Grégory Romé - gregory.rome@maxim-ic.com
:Organization: Maxim Integrated Products
:Copyright: Copyright © 2009-2018, Maxim Integrated Products
:License: BSD License - http://www.opensource.org/licenses/bsd-license.php

"""

import glob
import os

import serial


def scan():
    """
    Return a name list of available serial ports
    """

    available = []

    if os.name == 'posix':
        available = glob.glob('/dev/ttyS*') + glob.glob('/dev/ttyUSB*') + glob.glob('/dev/ttyACM*')
    else:
        for i in range(256):
            try:
                s = serial.Serial(i)
                available.append(s.portstr)
                s.close()   # explicit close 'cause of delayed GC in java
            except serial.SerialException:
                pass
    return available

if __name__ == '__main__':
    print scan()
