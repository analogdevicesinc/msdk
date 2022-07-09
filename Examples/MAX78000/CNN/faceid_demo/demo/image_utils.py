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
"""Functions for image processing
"""

import copy
import numpy as np
from PyQt5.QtGui import QImage


def cvt_img_to_qimage(img):
    """Converts OpenCv image to PyQt QImage
    """
    height, width, num_channles = img.shape
    bytes_per_line = num_channles * width
    return QImage(img.data, width, height, bytes_per_line, QImage.Format_RGB888)


def cvt_qimage_to_img(q_img, share_memory=False):
    """ Creates a numpy array from a QImage.

            If share_memory is True, the numpy array and the QImage is shared.
            Be careful: make sure the numpy array is destroyed before the image,
            otherwise the array will point to unreserved memory!!
    """
    assert isinstance(q_img, QImage), "img must be a QtGui.QImage object"
    assert q_img.format() == QImage.Format_RGB32, "img format must be QImage.Format.Format_RGB32, got: {}".format(q_img.format()) #pylint: disable=line-too-long

    img_size = q_img.size()
    buffer = q_img.constBits()
    buffer.setsize(img_size.height() * img_size.width() * 4)

    # Sanity check
    n_bits_buffer = len(buffer) * 8
    n_bits_image = img_size.width() * img_size.height() * q_img.depth()
    assert n_bits_buffer == n_bits_image, \
        "size mismatch: {} != {}".format(n_bits_buffer, n_bits_image)

    assert q_img.depth() == 32, "unexpected image depth: {}".format(q_img.depth())

    # Note the different width height parameter order!
    arr = np.ndarray(shape=(img_size.height(), img_size.width(), q_img.depth() // 8),
                     buffer=buffer,
                     dtype=np.uint8)
    arr = arr[:, :, :3]

    if share_memory:
        return arr

    return copy.deepcopy(arr)
