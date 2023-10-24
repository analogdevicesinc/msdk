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
"""Includes face identifier class to decide on person for given embedding
"""
import time
import numpy as np

from ai85_adapter import AI85SimulatorAdapter, AI85UartAdapter
from utils import load_data_arrs


class FaceID:
    """
    Class to identify embedding as either one of the faces in embeddings db or unknown.
    """
    face_db = None
    ai85_adapter = None
    thresh_for_unknowns = 0.0

    def __init__(self, face_db_path, unknown_threshold=0.0, ai85_adapter=None):
        '''
        :param face_db_path: Path to the face db file.
        :param unknown_threshold: Distance threshold to identify subjects who are not in the db.
        :param ai85_adapter: Type of the adapter used. Options: {None, 'sim', 'uart'}
        (default: None).
        '''
        subject_names_list, self.subj_list, self.embedding_list, self.img_list = load_data_arrs(face_db_path, #pylint: disable=line-too-long
                                                                                                load_img_prevs=True) #pylint: disable=line-too-long
        self.subj_ids = np.array(list(range(len(subject_names_list))))

        self.set_unknown_threshold(unknown_threshold)
        self.set_ai85_adapter(ai85_adapter)

    def has_ai85_adapter(self):
        """Checks if AI85 adapter is set"""
        if self.ai85_adapter is None:
            return False
        return True

    def __release_ai85_adapter(self):
        if self.ai85_adapter:
            del self.ai85_adapter
            self.ai85_adapter = None

    def __get_sorted_mean_distances_to_subjects(self, embedding):
        dists = embedding.astype(np.float32) - np.array(self.embedding_list.astype(np.float32))
        dists = np.sqrt(np.sum(np.square(dists), axis=1))

        mean_dists = np.zeros((len(self.subj_ids), ), dtype=np.float32)

        for i, sid in enumerate(self.subj_ids):
            mean_dists[i] = dists[self.subj_list == sid].mean()

        idx = np.argsort(mean_dists)
        mean_dists = mean_dists[idx]
        subjects = self.subj_ids[idx]

        return subjects, mean_dists

    def __get_min_mean_distance_subject(self, embedding):
        subjects, mean_dists = self.__get_sorted_mean_distances_to_subjects(embedding)
        return subjects[0], mean_dists[0]

    def set_unknown_threshold(self, unknown_threshold):
        """Sets threshold for unknown decision"""
        self.thresh_for_unknowns = unknown_threshold

    def set_ai85_adapter(self, adapter, **adapter_params):
        """Sets AI85 adapter"""
        if adapter is None:
            self.__release_ai85_adapter()
        elif adapter.lower() == 'sim':
            self.__release_ai85_adapter()
            if 'model_path' in adapter_params:
                self.ai85_adapter = AI85SimulatorAdapter(adapter_params['model_path'])
            else:
                print('Path to model checkpoint file should be declared as "model_path"!!!')
            self.__validate_db()
            print('Simulator activated!!')
        elif adapter.lower() == 'uart':
            self.__release_ai85_adapter()
            self.ai85_adapter = AI85UartAdapter(adapter_params['uart_port'],
                                                adapter_params['baud_rate'],
                                                adapter_params['embedding_len'])
            print('UART connection activated!!')
        else:
            print('Unknown AI85 Source selection')

    def __validate_db(self):
        fail = False
        for i, img in enumerate(self.img_list):
            emb = self.ai85_adapter.get_network_out(img)
            emb = emb[:, :, 0, 0].astype(np.int8).flatten()
            if np.sum(emb != self.embedding_list[i]) > 0:
                print(f'DB Validation Error: Issue in sample {i}')
                fail = True

        if not fail:
            print('Success in DB validation!')

    def run(self, img):
        """Runs face identifier."""
        subject_id = -1
        dist = -1
        ai85_time = 0
        db_match_time = 0

        if self.ai85_adapter:
            t_start = time.time()
            embedding = self.ai85_adapter.get_network_out(img)
            embedding = np.squeeze(embedding)
            t_get_emb = time.time()
            subject_id, dist = self.__get_sorted_mean_distances_to_subjects(embedding)

            unknown_idx = np.where(dist > self.thresh_for_unknowns)[0]
            if unknown_idx.size > 0:
                subject_id = np.insert(subject_id, unknown_idx[0], -1)
                dist = np.insert(dist, unknown_idx[0], self.thresh_for_unknowns)
            else:
                subject_id = np.append(subject_id, -1)
                dist = np.append(dist, self.thresh_for_unknowns)
            t_db_match = time.time()

            ai85_time = t_get_emb - t_start
            db_match_time = t_db_match - t_get_emb

        return subject_id, dist, ai85_time, db_match_time
