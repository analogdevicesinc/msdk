#!/usr/bin/env python3
###################################################################################################
################################################################################
 # Copyright (C) 2023 Maxim Integrated Products, Inc., All Rights Reserved.
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

"""This is the demo application for Face Identification which utilizes
MAX78000 EvKit to get CNN model output.
"""
import argparse
import numpy as np

from PyQt5 import QtWidgets
from PyQt5.QtGui import QImage, QPixmap, QIcon
from PyQt5.QtCore import pyqtSlot, Qt

import cv2

from camera import Camera
from cam_thread import Thread
from image_utils import cvt_qimage_to_img, cvt_img_to_qimage
from face_identifier import FaceID
from utils import load_data_arrs, get_face_image
from mtcnn.mtcnn import MTCNN


class FaceIdWindow(QtWidgets.QMainWindow):
    """
    Main window for FaceID demo.
    """

    # pylint: disable=too-many-instance-attributes

    cam_num = 0
    frame_size = (240, 320)
    img_size = (480, 640, 3)
    capture_size = (120, 160)
    frame_rate = 25
    model_params = {'embedding_len': 512,
                    'unknown_threshold': (1.0*560.0)}
    uart_params = {'port': 'COM10', 'baud_rate': 8*115200}
    db_paths = {'embeddings': 'embeddings.bin'}

    camera = None
    face_detector = None

    def __init__(self, com_port): #pylint: disable=too-many-statements
        super().__init__()
        self.uart_params['port'] = com_port

        self.setWindowTitle('AI-85 Powered Face ID')
        self.setWindowIcon(QIcon('logo.png'))

        self.__load_subject_map(self.db_paths['embeddings'])

        self.face_identifier = FaceID(face_db_path=self.db_paths['embeddings'],
                                      unknown_threshold=self.model_params['unknown_threshold'])

        #create GUI screen
        self.central_widget = QtWidgets.QWidget()

        #create dropdown menu to select environment for model execution
        self.ai85_source_label = QtWidgets.QLabel('AI-85 Source: ', self.central_widget)
        self.ai85_source_combo = QtWidgets.QComboBox(self.central_widget)
        self.ai85_source_combo.addItem('<Select AI85 Device>')
        self.ai85_source_combo.addItem('Simulator')
        self.ai85_source_combo.addItem('Emulator')
        self.ai85_source_combo.addItem('EV-Kit')
        self.ai85_source_combo.activated.connect(self.__source_selected)
        self.ai85_source_combo.setCurrentIndex(3)
        self.__source_selected()
        self.ai85_source_combo.setDisabled(True)

        #create button to load image
        self.load_img_button = QtWidgets.QPushButton('Load Image', self.central_widget)
        self.load_img_button.resize(100, 32)
        self.load_img_button.clicked.connect(self.__load_image)

        #create button to start camera
        self.start_camera_button = QtWidgets.QPushButton('Start Cam', self.central_widget)
        self.start_camera_button.resize(100, 32)
        self.start_camera_button.clicked.connect(self.__init_camera)

        #create button to capture image
        self.capture_button = QtWidgets.QPushButton('Capture', self.central_widget)
        self.capture_button.resize(100, 32)
        self.capture_button.clicked.connect(self.__capture_button_pressed)
        self.capture_button.setVisible(False)
        self.capture_busy = False

        # create button to stop camera
        self.stop_camera_button = QtWidgets.QPushButton('Stop Cam', self.central_widget)
        self.stop_camera_button.resize(100, 32)
        self.stop_camera_button.clicked.connect(self.__stop_camera)
        self.stop_camera_button.setVisible(False)

        #create view to show camera stream
        self.preview_frame = QtWidgets.QLabel('Preview', self.central_widget)
        self.preview_frame.resize(self.img_size[0], self.img_size[1])
        self.preview_black_img = QPixmap(self.img_size[1], self.img_size[0])
        self.preview_black_img.fill(Qt.black)
        self.preview_black_img = self.preview_black_img.toImage()
        self.__set_preview_image(self.preview_black_img)

        #create view to show captures frame
        self.captured_frame = QtWidgets.QLabel('Capture', self.central_widget)
        self.captured_frame.resize(self.capture_size[0], self.capture_size[1])
        black_img = QPixmap(self.capture_size[0], self.capture_size[1])
        black_img.fill(Qt.black)
        black_img = black_img.toImage()
        self.__set_captured_image(black_img)

        #create the text box to show results
        self.subject_label = QtWidgets.QLabel('Subject: ', self.central_widget)
        self.ai85_time_label = QtWidgets.QLabel('AI-85 Dur (ms): ', self.central_widget)
        self.db_time_label_label = QtWidgets.QLabel('DB Match Dur (ms): ', self.central_widget)
        self.ai85_energy_label = QtWidgets.QLabel('Energy (uJ): ', self.central_widget)

        self.subject_text = QtWidgets.QLabel('     ', self.central_widget)
        self.ai85_time_text = QtWidgets.QLabel('     ', self.central_widget)
        self.db_time_label_text = QtWidgets.QLabel('     ', self.central_widget)
        self.ai85_energy_text = QtWidgets.QLabel('     ', self.central_widget)

        self.result_box = QtWidgets.QGridLayout()
        self.result_box.addWidget(self.subject_label, 0, 0)
        self.result_box.addWidget(self.subject_text, 0, 1)
        self.result_box.addWidget(self.ai85_time_label, 1, 0)
        self.result_box.addWidget(self.ai85_time_text, 1, 1)
        self.result_box.addWidget(self.db_time_label_label, 2, 0)
        self.result_box.addWidget(self.db_time_label_text, 2, 1)
        self.result_box.addWidget(self.ai85_energy_label, 3, 0)
        self.result_box.addWidget(self.ai85_energy_text, 3, 1)

        font = self.subject_text.font()
        font.setBold(True)

        self.result_table = QtWidgets.QTableWidget(self.central_widget)
        self.result_table.verticalScrollBar().setDisabled(True)
        self.result_table.verticalScrollBar().setVisible(False)
        self.result_table.horizontalScrollBar().setDisabled(True)
        self.result_table.horizontalScrollBar().setVisible(False)
        self.result_table.setRowCount(4)
        self.result_table.setColumnCount(2)
        self.result_table.verticalHeader().setVisible(False)
        self.result_table.horizontalHeader().setVisible(False)
        for i in range(2):
            self.result_table.horizontalHeader().setSectionResizeMode(i,
                                                                      QtWidgets.QHeaderView.Stretch)
        for i in range(4):
            self.result_table.verticalHeader().setSectionResizeMode(i,
                                                                    QtWidgets.QHeaderView.Stretch)
        self.result_table.setItem(0, 0, QtWidgets.QTableWidgetItem("Subject"))
        self.result_table.setItem(0, 1, QtWidgets.QTableWidgetItem("Probability (%)"))
        self.top1_subj_text = QtWidgets.QTableWidgetItem("")
        self.top1_subj_dist = QtWidgets.QTableWidgetItem("")
        self.top2_subj_text = QtWidgets.QTableWidgetItem("")
        self.top2_subj_dist = QtWidgets.QTableWidgetItem("")
        self.top3_subj_text = QtWidgets.QTableWidgetItem("")
        self.top3_subj_dist = QtWidgets.QTableWidgetItem("")
        self.result_table.setItem(1, 0, self.top1_subj_text)
        self.result_table.setItem(1, 1, self.top1_subj_dist)
        self.result_table.setItem(2, 0, self.top2_subj_text)
        self.result_table.setItem(2, 1, self.top2_subj_dist)
        self.result_table.setItem(3, 0, self.top3_subj_text)
        self.result_table.setItem(3, 1, self.top3_subj_dist)

        self.result_table.item(0, 0).setFont(font)
        self.result_table.item(0, 1).setFont(font)
        font.setBold(False)
        self.result_table.item(1, 0).setFont(font)
        self.result_table.item(1, 1).setFont(font)
        self.result_table.item(2, 0).setFont(font)
        self.result_table.item(1, 1).setFont(font)
        self.result_table.item(3, 0).setFont(font)
        self.result_table.item(3, 1).setFont(font)

        #create layout that includes the ui components
        self.layout = QtWidgets.QGridLayout(self.central_widget)

        self.layout.addWidget(self.ai85_source_label, 0, 0)
        self.layout.addWidget(self.ai85_source_combo, 0, 1)

        self.layout.addWidget(self.load_img_button, 0, 4)
        self.layout.addWidget(self.start_camera_button, 0, 5)
        self.layout.addWidget(self.capture_button, 0, 5)
        self.layout.addWidget(self.stop_camera_button, 1, 5)

        self.layout.addWidget(self.preview_frame, 2, 0, 21, 6)
        self.layout.addWidget(self.captured_frame, 2, 7, 8, 4)

        self.layout.addLayout(self.result_box, 10, 6, 4, 3)
        self.layout.addWidget(self.result_table, 14, 6, 6, 4)

        self.setCentralWidget(self.central_widget)

    def __init_camera(self):
        self.start_camera_button.setText('Starting...')

        if self.camera is None:
            self.camera = Camera(cam_num=self.cam_num, frame_size=self.frame_size)

            # run thread to get frames from camera
            self.thread = Thread(parent=None, camera=self.camera, frame_rate=self.frame_rate)
            #self.thread.change_pixmap.connect(lambda p: self.__set_preview_image(p))
            self.thread.change_pixmap.connect(self.__set_preview_image)
            self.thread.start()

            self.start_camera_button.setVisible(False)
            self.load_img_button.setVisible(False)
            self.capture_button.setVisible(True)
            self.stop_camera_button.setVisible(True)

        self.start_camera_button.setText('Start Cam')

    def __stop_camera(self):
        if self.camera is not None:
            self.thread.terminate()
            self.camera.close_camera()

            self.__set_preview_image(self.preview_black_img)

            self.start_camera_button.setVisible(True)
            self.capture_button.setVisible(False)
            self.stop_camera_button.setVisible(False)
            self.load_img_button.setVisible(True)

            del self.camera
            self.camera = None

    def __load_image(self):
        if not self.face_identifier.has_ai85_adapter():
            self.__show_adapter_error()
            return

        img_path, _ = QtWidgets.QFileDialog.getOpenFileName(self, 'Open file', directory='',
                                                            filter="Image files (*.jpg *.jpeg *.bmp *.png) ") #pylint: disable=line-too-long
        if img_path == '':
            return

        print(img_path)
        with open(img_path, 'rb') as img_file:
            content = img_file.read()

        img = QImage()
        img.loadFromData(content)
        img_np = cvt_qimage_to_img(img)

        self.__set_preview_image(img.scaled(self.img_size[1], self.img_size[0],
                                            aspectRatioMode=Qt.KeepAspectRatio))
        if self.face_detector is None:
            self.face_detector = MTCNN(image_size=80, margin=0, min_face_size=60,
                                       thresholds=[0.6, 0.8, 0.92], factor=0.85,
                                       post_process=True, device='cpu')

        img = get_face_image(img_np, self.face_detector)
        if img is not None:
            if img.shape == (160, 120, 3):
                img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
                self.__set_captured_image(cvt_img_to_qimage(img))
                self.__identify_face(img)

    def __load_subject_map(self, path):
        subject_names_list, _, _, _ = load_data_arrs(path, load_img_prevs=False)

        self.subj_name_map = {}
        for i, subj_name in enumerate(subject_names_list):
            self.subj_name_map[i] = subj_name
        self.subj_name_map[-1] = 'Unknown'

        print(self.subj_name_map)

    def __source_selected(self):
        if self.ai85_source_combo.currentIndex() == 0:
            self.face_identifier.set_ai85_adapter(None)
        elif self.ai85_source_combo.currentIndex() == 1:
            self.face_identifier.set_ai85_adapter('sim', model_path=self.model_params['path'])
        elif self.ai85_source_combo.currentIndex() <= 3:
            self.face_identifier.set_ai85_adapter('uart', uart_port=self.uart_params['port'],
                                                  baud_rate=self.uart_params['baud_rate'],
                                                  embedding_len=self.model_params['embedding_len'])
        else:
            print('Unknown AI85 Source selection')

    def __show_adapter_error(self):
        err_msg = QtWidgets.QErrorMessage(self)
        err_msg.setWindowTitle("Adapter Error")
        err_msg.showMessage('No AI-85 Adapter!!')
        _ = err_msg.exec_()

    def __capture_button_pressed(self):
        if not self.capture_busy:
            self.capture_busy = True

            if not self.face_identifier.has_ai85_adapter():
                self.__show_adapter_error()
                return

            capture = self.preview_frame.pixmap()
            capture = capture.toImage()

            captured_img = cvt_qimage_to_img(capture)

            cropped_img = captured_img[self.camera.start_point[1]:self.camera.end_point[1],
                                       self.camera.start_point[0]:self.camera.end_point[0]]
            cropped_img = cv2.resize(cropped_img, self.capture_size)
            cropped_img = cv2.cvtColor(cropped_img, cv2.COLOR_BGR2RGB).copy()

            self.__set_captured_image(cvt_img_to_qimage(cropped_img))
            self.__identify_face(cropped_img)

            self.capture_busy = False

    def __identify_face(self, cropped_img):
        # identify face
        subject_id, dist, ai85_time, db_match_time = self.face_identifier.run(cropped_img)

        prob = self.__cvt_dist_to_prob(dist)

        self.subject_text.setText(self.subj_name_map[subject_id[0]])
        self.ai85_time_text.setText('%.3f' % (1000 * ai85_time))
        self.db_time_label_text.setText('%.3f' % (1000 * db_match_time))
        self.ai85_energy_text.setText('N/A')

        self.top1_subj_text.setText(self.subj_name_map[subject_id[0]])
        self.top1_subj_dist.setText('%.2f' % prob[0])
        self.top2_subj_text.setText(self.subj_name_map[subject_id[1]])
        self.top2_subj_dist.setText('%.2f' % prob[1])
        self.top3_subj_text.setText(self.subj_name_map[subject_id[2]])
        self.top3_subj_dist.setText('%.2f' % prob[2])

    def __cvt_dist_to_prob(self, dist): #pylint: disable=no-self-use
        prob = 1.0 / np.power(dist, 4)
        prob /= np.sum(prob)
        prob *= 100.0
        return prob


    @pyqtSlot(QImage)
    def __set_preview_image(self, image):
        self.preview_frame.setPixmap(QPixmap.fromImage(image))

    def __set_captured_image(self, image):
        self.captured_frame.setPixmap(QPixmap.fromImage(image))

    def __del__(self):
        self.__stop_camera()


def run_face_id_app():
    """
    Main function to initiate demo GUI.
    """
    parser = argparse.ArgumentParser()
    parser.add_argument('--uart_com_port', '-c', type=str, help='com port for UART communication')
    args = parser.parse_args()

    try:
        app = QtWidgets.QApplication([])
        app_window = FaceIdWindow(args.uart_com_port)
        app_window.show()
        app.exit(app.exec_())
    except Exception as ex: #pylint: disable=broad-except
        template = "An exception of type {0} occurred. Arguments:\n{1!r}"
        message = template.format(type(ex).__name__, ex.args)
        print(message)


if __name__ == "__main__":
    run_face_id_app()
