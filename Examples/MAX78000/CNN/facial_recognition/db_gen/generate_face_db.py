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
