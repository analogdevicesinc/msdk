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
Utility functions to generate embeddings and I/O operations
"""

import os
from collections import defaultdict
import numpy as np


from cv2 import imread
from PIL import Image, ExifTags
import torch
import torchvision.transforms.functional as VF


def get_image_rotation(img_path):
    """Reads exif data of the image to get image orientation
    """
    for orientation in ExifTags.TAGS:
        if ExifTags.TAGS[orientation] == 'Orientation':
            break

    image = Image.open(img_path)
    try:
        exif = image._getexif()  # pylint: disable=protected-access
        if exif:
            if orientation in exif:
                return exif[orientation]
    except Exception:  # pylint: disable=broad-except
        print(f'No exif object for {img_path}')

    return 1


def rotate_image(img, img_rot):
    """Rotates image wrt `orientation` value of the exif data
    """
    if img_rot == 1:
        pass
    elif img_rot == 6:
        return np.rot90(img, k=3).copy()
    elif img_rot == 8:
        return np.rot90(img, k=1).copy()
    elif img_rot == 3:
        return np.rot90(img, k=2).copy()
    else:
        print(f'Unknown orientation code: {img_rot}. Image will be used as is.')

    return img


def append_db_file_from_path(folder_path, face_detector, ai85_adapter):
    """Creates embeddings for each image in the given folder and appends to the existing embedding
    dictionary
    """
    subj_id = 0
    subject_list = sorted(os.listdir(folder_path))
    emb_array = np.zeros([1024, 64]).astype(np.uint8)
    recorded_subject = []
    emb_id = 0
    output_shift = 2 #TODO: Check here for adj. output shift
    summary = {}

    for i in subject_list:
        summary[i] = 0
    for subject in subject_list:
        print(f'Processing subject: {subject}')
        
        subject_path = os.path.join(folder_path, subject)
        if not os.path.isdir(subject_path):
            continue
        if not os.listdir(subject_path):
            subj_id += 1
        for file in os.listdir(subject_path):
            print(f'\tFile: {file}')
            img_path = os.path.join(subject_path, file)
            img_rot = get_image_rotation(img_path)
            img = imread(img_path)
            img = rotate_image(img, img_rot)
            img = img.astype(np.float32)            

            img = get_face_image(img, face_detector)
            if img is not None:
                img = ((img+1)*128)
                img = (img.squeeze()).detach().cpu().numpy()
                img = img.astype(np.uint8)
                img = img.transpose([1, 2, 0])

                if img.shape == (112, 112, 3):
                    current_embedding = ai85_adapter.get_network_out(img)[0, :]
                    current_embedding = current_embedding * 128 * output_shift #convert back to 8 bits after normalization
                    current_embedding = np.clip(current_embedding, -128, 127)  #clamp if embedding > 0.5 or < -0.5 as shift = 2
                    current_embedding[current_embedding < 0] += 256 # Convert negatives to uint
                    current_embedding = np.around(current_embedding).astype(np.uint8).flatten()

                    emb_array[emb_id,:] = current_embedding
                    recorded_subject.append(subject)
                    emb_id += 1
                    summary[subject] += 1
                                      
    #np.save('emb_array.npy', emb_array)
    print('Database Summary')
    for key in summary:
        print(f'\t{key}:', f'{summary[key]} images')
    #Format summary for printing image counts per subject
    
        

    return emb_array, recorded_subject
def get_face_image(img, face_detector):
    """Detects face on the given image
    """
    device = torch.device('cuda:0' if torch.cuda.is_available() else 'cpu')
    bboxes, lndmrks = face_detector.detect(img)
    try:
        pbox = bboxes[0]
    except IndexError:
        print('No face detected')
        return None
    img = torch.Tensor(img.transpose([2, 0, 1])).to(device).unsqueeze(0)
    
    img = Normalize_Img(img) #Normalize image for faceID
        
    # Convert bounding box to square
    height = pbox[3] - pbox[1]
    width = pbox[2] - pbox[0]
    max_dim = max(height, width)
    pbox[0] = np.clip(pbox[0] - (max_dim - width) / 2, 0, img.shape[3])
    pbox[1] = np.clip(pbox[1] - (max_dim - height) / 2, 0, img.shape[2])
    pbox[2] = np.clip(pbox[2] + (max_dim - width) / 2, 0, img.shape[3])
    pbox[3] = np.clip(pbox[3] + (max_dim - height) / 2, 0, img.shape[2])

    # Crop image with the square bounding box
    img = VF.crop(img=img, top=int(pbox[1]), left=int(pbox[0]),
                  height=int(pbox[3]-pbox[1]), width=int(pbox[2]-pbox[0]))
            
    # Check if the cropped image is square, if not, pad it

    _, _, h, w = img.shape
    if w != h:
        max_dim = max(w, h)
        h_padding = (max_dim - h) / 2
        w_padding = (max_dim - w) / 2
        l_pad = w_padding if w_padding % 1 == 0 else w_padding+0.5
        t_pad = h_padding if h_padding % 1 == 0 else h_padding+0.5
        r_pad = w_padding if w_padding % 1 == 0 else w_padding-0.5
        b_pad = h_padding if h_padding % 1 == 0 else h_padding-0.5
        padding = (int(l_pad), int(t_pad), int(r_pad), int(b_pad))
        img = VF.pad(img, padding, 0, 'constant')
            
    # Resize cropped image to the desired size
    img = VF.resize(img, (112, 112))

    return img

def create_baseaddr_include_file(baseaddr_h_path):

    data_arr = []
    data_line = []


    baseaddr= ["0x50180000", "0x50184000", "0x50188000", "0x5018c000", "0x50190000", "0x50194000", "0x50198000", "0x5019c000",
        "0x501a0000", "0x501a4000", "0x501a8000", "0x501ac000", "0x501b0000", "0x501b4000", "0x501b8000", "0x501bc000",
        "0x50580000", "0x50584000", "0x50588000", "0x5058c000", "0x50590000", "0x50594000", "0x50598000", "0x5059c000",
        "0x505a0000", "0x505a4000", "0x505a8000", "0x505ac000", "0x505b0000", "0x505b4000", "0x505b8000", "0x505bc000",
        "0x50980000", "0x50984000", "0x50988000", "0x5098c000", "0x50990000", "0x50994000", "0x50998000", "0x5099c000",
        "0x509a0000", "0x509a4000", "0x509a8000", "0x509ac000", "0x509b0000", "0x509b4000", "0x509b8000", "0x509bc000",
        "0x50d80000", "0x50d84000", "0x50d88000", "0x50d8c000", "0x50d90000", "0x50d94000", "0x50d98000", "0x50d9c000",
        "0x50da0000", "0x50da4000", "0x50da8000", "0x50dac000", "0x50db0000", "0x50db4000", "0x50db8000", "0x50dbc000"]

    if baseaddr_h_path is not None:
        with open(baseaddr_h_path, 'w') as h_file:
            h_file.write('#define BASEADDR { \\\n  ')
            for base in baseaddr:
                data_line.append(base)
                if (len(data_line) % 8) == 0:
                    data_arr.append(', '.join(data_line))
                    data_line.clear()

            data_arr.append(', '.join(data_line))
            data = ', \\\n  '.join(data_arr)
            h_file.write(data)
            h_file.write(' \\\n}')
            h_file.write('\n')

    return baseaddr

def create_weights_include_file(emb_array, weights_h_path, baseaddr):
    """
    Create weights_3.h file from embeddings
    """
    Embedding_dimension = 64
    extension = os.path.splitext(weights_h_path)[1]

    
    length = "0x00000101"
    data_arr = []
    data_line = []
    
    
    if extension == '.h':
        with open(weights_h_path, 'w') as h_file:
            four_byte = []
            h_file.write('#define KERNELS_3 { \\\n  ')
            for dim in range(Embedding_dimension):
                init_proccessor = False
                for i in range(emb_array.shape[0] + 4): # nearest %9 == 0 for 1024 is 1027, it can be kept in 1028 bytes TODO: Change this from Hardcoded
                    reindex = i + 8 - 2*(i%9)
                    if reindex < 1024: # Total emb count is 1024, last index 1023
                        single_byte = str(format(emb_array[reindex][dim], 'x')) #Relocate emb for cnn kernel
                    else:
                        single_byte = str(format(0, 'x'))
                    if len(single_byte) == 1:
                        single_byte = '0' + single_byte
                    four_byte.append(single_byte)

                    if (i + 1) % 4 == 0:
                        if not init_proccessor:
                            data_line.append(baseaddr[dim])
                            if (len(data_line) % 8) == 0:
                                data_arr.append(', '.join(data_line))
                                data_line.clear()
                            data_line.append(length)
                            if (len(data_line) % 8) == 0:
                                data_arr.append(', '.join(data_line))
                                data_line.clear()
                            init_proccessor = True
                        data_32 = '0x'+''.join(four_byte)

                        data_line.append(data_32)
                        four_byte.clear()
                        if (len(data_line) % 8) == 0:
                            data_arr.append(', '.join(data_line))
                            data_line.clear()
        
            data_arr.append(', '.join(data_line))
            data = ', \\\n  '.join(data_arr)
            h_file.write(data)
            h_file.write('0x00000000') #TODO: CHECK THE SOURCE OF THE LAST 0x00000000, PS: Might be due to addr matching while loading weights at c side
            h_file.write(' \\\n}')
            h_file.write('\n')

    elif extension == '.bin':
        with open(weights_h_path, 'wb') as h_file:
            four_byte = 0
            data_arr = bytearray(np.uint8([data_arr]))
            for dim in range(Embedding_dimension):
                for i in range(emb_array.shape[0] + 4): # nearest %9 == 0 for 1024 is 1027, it can be kept in 1028 bytes TODO: Change this from Hardcoded
                    reindex = i + 8 - 2*(i%9)
                    if reindex < 1024: # Total emb count is 1024, last index 1023
                        single_byte = int(emb_array[reindex][dim])  #Relocate emb for cnn kernel
                    else:
                        single_byte = 0
                    four_byte = four_byte << 8 | single_byte
                    if (i + 1) % 4 == 0:
                        data_arr.extend(four_byte.to_bytes(4, 'little', signed=False))
                        four_byte = 0
            h_file.write(data_arr)


def create_embeddings_include_file(recorded_subjects, embeddings_h_path):
    data_arr = []
    data_line = []

    with open(embeddings_h_path, 'w') as h_file:
        h_file.write('#define DEFAULT_EMBS_NUM ' + str(len(recorded_subjects)) + ' \n')
        h_file.write('\n')
        h_file.write('#define DEFAULT_NAMES { \\\n ')

        for subject in recorded_subjects:
            if len(subject) > 6: #TODO: 6 is the max name length for now
                subject = subject[:6]
            data_line.append('"' + subject + '"')
            if (len(data_line) % 15) == 0:
                data_arr.append(', '.join(data_line))
                data_line.clear()

        data_arr.append(', '.join(data_line))
        data = ', \\\n '.join(data_arr)
        h_file.write(data)
        h_file.write(' \\\n}')
        h_file.write('\n')

def Normalize_Img(img):
    return img.sub(128).clamp(min=-128, max=127).div(128.)
