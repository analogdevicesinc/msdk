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
"""Includes camera class to capture image
"""
import cv2


class Camera:
    """Captures image from selected camera and returns image inside drawed rectangle"""
    def __init__(self, cam_num=0, frame_size=(240, 320)):
        self.cap = cv2.VideoCapture(cam_num)
        self.cam_num = cam_num

        width = int(self.cap.get(3))  # int
        height = int(self.cap.get(4))  # int
        print(f'Webcam image size: ({width}, {height})')  # webcam_size

        frame_width = frame_size[0]
        frame_height = frame_size[1]

        self.thickness = 3
        self.start_point = ((width - frame_width) // 2, (height - frame_height) // 2)
        self.end_point = (self.start_point[0] + frame_width, self.start_point[1] + frame_height)
        self.color = (255, 0, 0)

    def get_frame(self):
        """Returns the next captured image"""
        rval, frame = self.cap.read()
        if rval:
            frame = cv2.rectangle(frame, tuple(q-self.thickness for q in self.start_point),
                                  tuple(q+self.thickness for q in self.end_point),
                                  self.color, self.thickness)
            frame = cv2.flip(frame, 1)
            frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)

        return rval, frame

    def close_camera(self):
        """Releases camera"""
        self.cap.release()

    def __str__(self):
        return f'OpenCV Camera {self.cam_num}'
