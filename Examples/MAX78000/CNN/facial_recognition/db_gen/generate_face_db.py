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

"""
Script to generate Face Id embeddings
"""
import argparse
import numpy as np
import os
import os.path as path
from ai85.ai85_adapter import AI85SimulatorAdapter
from hawk_eyes.face import RetinaFace

from utils import append_db_file_from_path, create_weights_include_file, create_embeddings_include_file, create_baseaddr_include_file

CURRENT_DIR = path.abspath(path.dirname(path.abspath(__file__)))
MODEL_PATH = path.join(CURRENT_DIR, 'model', 'ai85-faceid_112-qat-q.pth.tar')


def create_db_from_folder(args):
    """
    Main function of the script to generate face detector, AI85 simulator and calls the utility
    functions to generate embeddings and store them in required format.
    """


    ai85_adapter = AI85SimulatorAdapter(MODEL_PATH)
    face_detector = RetinaFace(model_name='retina_l', conf=0.1)
    
    os.makedirs(args.db, exist_ok=True)

    emb_array, recorded_subject = append_db_file_from_path(args.db, face_detector, ai85_adapter)

    baseaddr = create_baseaddr_include_file(args.base)
    create_weights_include_file(emb_array, args.weights, baseaddr)
    create_embeddings_include_file(recorded_subject, args.emb)
    print(f'Created weights and embeddings files from {len(recorded_subject)} images.')


def parse_arguments():
    """
    Function to parse comman line arguments.
    """
    parser = argparse.ArgumentParser(description='Create embedding database file.')
    parser.add_argument('--db', '-db-path', type=str, default='db',
                        help='path for face images')
    parser.add_argument('--base', '-base-path', type=str, default='include\\baseaddr.h',
                        help='path for baseaddr header file')
    parser.add_argument('--emb', '-emb-path', type=str, default='include\embeddings.h',
                        help='path for embeddings header file')
    parser.add_argument('--weights', '-weights-path', type=str, default='include\weights_3.h',
                        help='path for weights header file')

    args = parser.parse_args()
    return args


def main():
    """
    Entry point of the script to parse command line arguments and run the function to generate
    embeddings.
    """
    args = parse_arguments()
    create_db_from_folder(args)


if __name__ == "__main__":
    main()
