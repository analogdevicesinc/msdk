#!/usr/bin/env python3
###################################################################################################
################################################################################
 # Copyright (C) 2022 Maxim Integrated Products, Inc., All Rights Reserved.
 #
 # Permission is hereby granted, free of charge, to any person obtaining a
 # copy of this software and associated documentation files (the "Software"),
 # to deal in the Software without restriction, including without limitation
 # the rights to use, copy, modify, merge, publish, distribute, sublicense,
 # and/or sell copies of the Software, and to permit persons to whom the
 # Software is furnished to do so, subject to the following conditions:
 #
 # The above copyright notice and this permission notice shall be included
 # in all copies or substantial portions of the Software.
 #
 # THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS
 # OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
 # MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.
 # IN NO EVENT SHALL MAXIM INTEGRATED BE LIABLE FOR ANY CLAIM, DAMAGES
 # OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE,
 # ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR
 # OTHER DEALINGS IN THE SOFTWARE.
 #
 # Except as contained in this notice, the name of Maxim Integrated
 # Products, Inc. shall not be used except as stated in the Maxim Integrated
 # Products, Inc. Branding Policy.
 #
 # The mere transfer of this software does not imply any licenses
 # of trade secrets, proprietary technology, copyrights, patents,
 # trademarks, maskwork rights, or any other form of intellectual
 # property whatsoever. Maxim Integrated Products, Inc. retains all
 # ownership rights.
 #
 ###############################################################################
"""Thread to capture image from camera continiously
"""
import time

from PyQt5.QtCore import (QThread, pyqtSignal, Qt)
from PyQt5.QtGui import QImage

from image_utils import cvt_img_to_qimage


class Thread(QThread):
    """
    Thread to capture image from camera
    """
    change_pixmap = pyqtSignal(QImage)

    def __init__(self, parent=None, camera=None, frame_rate=25):
        QThread.__init__(self, parent=parent)
        self.camera = camera
        self.emit_period = 1.0 / frame_rate

    def run(self):
        """Runs camera capture"""
        prev = time.time()
        while True:
            now = time.time()
            rval, frame = self.camera.get_frame()
            if rval:
                convert_qt_format = cvt_img_to_qimage(frame)
                qt_img = convert_qt_format.scaled(640, 480, Qt.KeepAspectRatio)
                if (now - prev) >= self.emit_period:
                    self.change_pixmap.emit(qt_img)
                    prev = now
