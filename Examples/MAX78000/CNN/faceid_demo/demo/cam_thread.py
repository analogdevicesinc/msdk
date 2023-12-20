#!/usr/bin/env python3
###################################################################################################
###############################################################################
 #
 # Copyright (C) 2022-2023 Maxim Integrated Products, Inc. All Rights Reserved.
 # (now owned by Analog Devices, Inc.),
 # Copyright (C) 2023 Analog Devices, Inc. All Rights Reserved. This software
 # is proprietary to Analog Devices, Inc. and its licensors.
 #
 # Licensed under the Apache License, Version 2.0 (the "License");
 # you may not use this file except in compliance with the License.
 # You may obtain a copy of the License at
 #
 #     http://www.apache.org/licenses/LICENSE-2.0
 #
 # Unless required by applicable law or agreed to in writing, software
 # distributed under the License is distributed on an "AS IS" BASIS,
 # WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 # See the License for the specific language governing permissions and
 # limitations under the License.
 #
 ##############################################################################
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
