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
Utility functions to generate embeddings and I/O operations
"""

import os
import csv
import copy
from collections import defaultdict
import numpy as np
import scipy
import scipy.ndimage
from matplotlib.image import imread
import matplotlib.pyplot as plt
from PIL import Image, ExifTags


def load_db(db_path, subj_name_file_path=None):
    """Loads embeddings from binary file to a dictionary
    """
    subj_ids, subj_list, embedding_list = load_embedding_list(db_path)

    if subj_name_file_path:
        subj_name_map = load_subject_map(subj_name_file_path)
    else:
        subj_name_map = {}
        for i in subj_ids:
            subj_name_map[i] = ('%d' % i)

    embedding_db = {}
    for i in range(subj_list.size):
        subj = subj_name_map[subj_list[i]]
        if subj not in embedding_db:
            embedding_db[subj] = {}
            emb_no = 1
        else:
            emb_no = len(embedding_db[subj]) + 1
        embedding_db[subj]['Embedding_%d' % emb_no] = {'emb': embedding_list[i, :], 'img': None}

    return embedding_db


def load_subject_map(path):
    """Loads subject names to a dictionary
    """
    subj_name_map = {}
    with open(path) as file:
        data = csv.reader(file, delimiter=',')
        for row in data:
            subj_name_map[int(row[0])] = row[1]

    return subj_name_map


def load_embedding_list(db_path):
    """Loads embeddings from binary file to lists
    """
    ##########################################
    # The data in order:
    #    1 byte : number of subjects   (S)
    #    2 bytes: length of embeddings (L)
    #    2 bytes: number of embeddings (N)
    #    (L+1)*N bytes: embeddings
    #        1 byte : subject id
    #        L bytes: embedding
    ##########################################

    subj_list = []
    embedding_list = []

    with open(db_path, "rb") as file:
        _ = int.from_bytes(file.read(1), byteorder='big', signed=False) # pylint: disable=invalid-name
        L = int.from_bytes(file.read(2), byteorder='big', signed=False) # pylint: disable=invalid-name
        N = int.from_bytes(file.read(2), byteorder='big', signed=False) # pylint: disable=invalid-name

        for _ in range(N):
            subj_list.append(int.from_bytes(file.read(1), byteorder='big', signed=False))
            embedding_list.append(list(file.read(L)))

    subj_list = np.array(subj_list)
    subj_ids = np.unique(subj_list)

    embedding_list = np.array(embedding_list)
    neg_idx = embedding_list > 127
    embedding_list[neg_idx] -= 256

    return subj_ids, subj_list, embedding_list


def equalize_hist(img, percentile=95):
    """Histogram equalization for a given image
    """
    dc = img.ravel().mean() # pylint: disable=invalid-name
    ac = img - dc # pylint: disable=invalid-name
    max_val = np.percentile(np.abs(ac.ravel()), percentile)
    mult = 127 / max_val
    new_img = np.clip(np.round(mult * ac + dc), 0, 255)
    return new_img.astype(np.uint8)


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


# pylint: disable=too-many-statements
def append_db_file_from_path(folder_path, mtcnn, ai85_adapter, db_dict=None, verbose=True,
                             preview_images=False):
    """Creates embeddings for each image in the given folder and appends to the existing embedding
    dictionary
    """
    existing_db_dict = db_dict
    db_dict = defaultdict(dict)
    img_list = []
    subj_id = 0
    subject_list = sorted(os.listdir(folder_path))
    for subject in subject_list:
        print(f'Processing subject: {subject}')
        img_id = 0
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
            if img.dtype == np.float32:
                img = (255 * img).astype(np.uint8)
            img = get_face_image(img, mtcnn)
            if img is not None:
                if img.shape == (160, 120, 3):
                    img_id += 1
                    img = equalize_hist(img, 99)
                    current_embedding = ai85_adapter.get_network_out(img)[:, :, 0, 0]
                    current_embedding = current_embedding.astype(np.int8).flatten()
                    img_list.append(img)
                    db_dict[subject]['Embedding_%d' % img_id] = {'emb': current_embedding,
                                                                 'img': img}

    # Summary and determination of max photo per user
    max_photo = 0
    if verbose:
        if existing_db_dict:
            print('New entries for the DB')
        else:
            print('A new DB with')
    for idx, subj in enumerate(db_dict.keys()):
        if verbose:
            print(f'\t{subj}: {len(db_dict[subj].keys())} images')
        if len(list(db_dict[subj].keys())) > max_photo:
            max_photo = len(list(db_dict[subj].keys()))
    if verbose:
        if existing_db_dict:
            print('have been appended!')
        else:
            print('has been created!')

    # Preview image formation
    preview = None
    if preview_images:
        preview = 125*np.ones((len(db_dict.keys()) * 160, max_photo * 120, 3))
        for idx, subj in enumerate(db_dict.keys()):
            start_y = 0 + idx * 160
            start_x = 0
            for img_ind in db_dict[subj].keys():
                preview[start_y:start_y+160, start_x:start_x+120, :] = db_dict[subj][img_ind]['img']
                start_x += 120
        preview = preview.astype(np.uint8)
        if verbose:
            plt.figure(figsize=(1.5*max_photo, 2*len(db_dict.keys())))
            plt.imshow(preview)
            plt.show()

    # Merge the new db and existing one if append mode is called
    if existing_db_dict:
        integrated_db = copy.deepcopy(existing_db_dict)
        for subj in db_dict.keys():
            # if same subject exists in both dictionaries
            if subj in existing_db_dict.keys():
                img_id = max(list(existing_db_dict[subj].keys())) + 1
                for ind in db_dict[subj].keys():
                    integrated_db[subj]['Embedding_%d' % img_id] = integrated_db[subj][ind]
                    img_id += 1
            else:
                integrated_db[subj] = integrated_db[subj]
        db_dict = integrated_db

    return db_dict, preview


def get_face_image(img, mtcnn, min_prob=0.8):
    """Detects face on the given image
    """
    _, prob, box = mtcnn(img, return_prob=True)
    if box is not None and prob > min_prob:
        img_arr, _, _, _ = generate_image(img, box, 1)
        return img_arr[0]

    return None


def generate_image(img, box, count):
    """Randomly crop 160x120 sub image around the given box.

    img: Original Image
    box: Rectangle in format x1, y1, x3, y3.
    count: Number of pics to generate
    """
    box[0] = np.max((box[0], 0))
    box[1] = np.max((box[1], 0))
    box[2] = np.min((box[2], img.shape[1]))
    box[3] = np.min((box[3], img.shape[0]))

    factor = 1
    height = img.shape[0]
    width = img.shape[1]
    new_img = img
    new_box = box
    while True:
        if height < 160 or width < 120:
            factor += 1
            new_img = scipy.ndimage.zoom(img, [factor, factor, 1], order=1)
            new_box = box * factor
            height = new_img.shape[0]
            width = new_img.shape[1]
        else:
            break
    new_box = np.round(new_box).astype(np.int)
    new_box_height = new_box[3] - new_box[1]
    new_box_width = new_box[2] - new_box[0]

    scale_list = np.concatenate((np.arange(0.9, 0, -0.1), np.arange(0.09, 0, -0.02)))
    ind = 0
    while (new_box_height > 160) or (new_box_width > 120):
        if ind < scale_list.size:
            new_img = scipy.ndimage.zoom(img, [scale_list[ind], scale_list[ind], 1], order=1)
            new_box = box * scale_list[ind]
            new_box = np.round(new_box).astype(np.int)
            new_box_height = new_box[3] - new_box[1]
            new_box_width = new_box[2] - new_box[0]
            ind += 1
        else:
            pass

    min_x = np.max((0, new_box[0] - (120 - new_box_width)))
    min_y = np.max((0, new_box[1] - (160 - new_box_height)))
    max_x = np.min((new_box[0], width - 120))
    max_y = np.min((new_box[1], height - 160))

    start_x = np.random.choice(np.arange(min_x, max_x + 1), count, replace=True)
    start_y = np.random.choice(np.arange(min_y, max_y + 1), count, replace=True)
    img_arr = []
    box_arr = []
    for i in range(count):
        img_arr.append(new_img[start_y[i]:start_y[i] + 160, start_x[i]:start_x[i] + 120].copy())
        temp_box = new_box.copy()
        temp_box[0] -= start_x[i]
        temp_box[2] -= start_x[i]
        temp_box[1] -= start_y[i]
        temp_box[3] -= start_y[i]
        box_arr.append(temp_box)
    new_img = img_arr
    new_box = box_arr
    return new_img, new_box, img, box


def create_data_arrs(emb_db, add_prev_imgs):
    """Converts embedding dict to lists
    """
    subject_names = list(emb_db.keys())

    subject_arr = []
    embedding_arr = []
    img_arr = []

    for i, subj in enumerate(emb_db.keys()):
        for emb_key in emb_db[subj].keys():
            subject_arr.append(i)
            embedding_arr.append(emb_db[subj][emb_key]['emb'])
            if add_prev_imgs:
                if emb_db[subj][emb_key]['img'] is not None:
                    img_arr.append(emb_db[subj][emb_key]['img'].flatten())
                else:
                    img_arr.append(np.zeros((160 * 120 * 3,), np.uint8))

    return subject_names, np.array(subject_arr), np.array(embedding_arr), np.array(img_arr)


def save_embedding_db(emb_db, db_path, add_prev_imgs=False):
    """
    Saves embedding database in binary format

    The data in order:
        1 byte : number of subjects (S)
        2 bytes: length of embeddings (L)
        2 bytes: number of embeddings (N)
        2 bytes: length of image width (W)
        2 bytes: length of image height (H)
        2 bytes: length of subject names (K)
        K bytes: subject names
        (L+1)*N bytes: embeddings
            1 byte : subject id
            L bytes: embedding
        (W*H*3)*N bytes: image
    """

    subject_names, subject_arr, embedding_arr, img_arr = create_data_arrs(emb_db, add_prev_imgs)

    subject_arr = subject_arr.astype(np.uint8)
    embedding_arr = embedding_arr.astype(np.int8)

    names_str = '  '.join(subject_names)
    names_bytes = bytearray()
    names_bytes.extend(map(ord, names_str))

    S = np.unique(subject_arr).size # pylint: disable=invalid-name
    K = len(names_bytes) # pylint: disable=invalid-name
    N, L = embedding_arr.shape # pylint: disable=invalid-name
    W = 120 # pylint: disable=invalid-name
    H = 160 # pylint: disable=invalid-name

    db_data = bytearray(np.uint8([S]))
    db_data.extend(L.to_bytes(2, 'big', signed=False))
    db_data.extend(N.to_bytes(2, 'big', signed=False))
    db_data.extend(W.to_bytes(2, 'big', signed=False))
    db_data.extend(H.to_bytes(2, 'big', signed=False))

    db_data.extend(K.to_bytes(2, 'big', signed=False))
    db_data.extend(names_bytes)

    for i, emb in enumerate(embedding_arr):
        db_data.extend(bytearray([subject_arr[i]]))
        db_data.extend(bytearray(emb))

    if add_prev_imgs:
        for img in img_arr:
            db_data.extend(bytearray(img))

    with open(db_path, 'wb') as file:
        file.write(db_data)

    print(f'Binary embedding file is saved to "{db_path}".')


def load_data_arrs(db_path, load_img_prevs=True):
    """Loads embeddings from binary file to lists
    """

    with open(db_path, "rb") as file:
        _ = int.from_bytes(file.read(1), byteorder='big', signed=False) # pylint: disable=invalid-name
        L = int.from_bytes(file.read(2), byteorder='big', signed=False) # pylint: disable=invalid-name
        N = int.from_bytes(file.read(2), byteorder='big', signed=False) # pylint: disable=invalid-name
        W = int.from_bytes(file.read(2), byteorder='big', signed=False) # pylint: disable=invalid-name
        H = int.from_bytes(file.read(2), byteorder='big', signed=False) # pylint: disable=invalid-name
        K = int.from_bytes(file.read(2), byteorder='big', signed=False) # pylint: disable=invalid-name

        subject_names_str = file.read(K).decode('ascii')
        subject_names_list = subject_names_str.split('  ')

        subj_list = []
        embedding_list = []

        for _ in range(N):
            subj_list.append(int.from_bytes(file.read(1), byteorder='big', signed=False))
            embedding_list.append(list(file.read(L)))

        embedding_list = np.array(embedding_list).astype(np.int8)
        neg_idx = embedding_list > 127
        embedding_list[neg_idx] -= 256

        img_arr = None
        if load_img_prevs:
            img_bin = file.read(N * W * H * 3)
            if img_bin:
                img_arr = np.array(list(img_bin)).astype(np.uint8)
                img_arr = img_arr.reshape(N, H, W, 3)

    return subject_names_list, subj_list, embedding_list, img_arr


def load_embedding_db(db_path):
    """
    Loads embeddings from binary file to dictionary

    The data in order:
        1 byte : number of subjects (S)
        2 bytes: length of embeddings (L)
        2 bytes: number of embeddings (N)
        2 bytes: length of image width (W)
        2 bytes: length of image height (H)
        2 bytes: length of subject names (K)
        K bytes: subject names
        (L+1)*N bytes: embeddings
            1 byte : subject id
            L bytes: embedding
        (W*H*3)*N bytes: image
    """

    subject_names_list, subj_list, embedding_list, img_arr = load_data_arrs(db_path)

    emb_db = {}
    for subj in subject_names_list:
        emb_db[subj] = {}

    for i, _ in enumerate(subj_list):
        subj_name = subject_names_list[subj_list[i]]
        emb = embedding_list[i]
        if img_arr is None:
            img = None
        else:
            img = img_arr[i, :, :, :]
        emb_name = 'Embedding_%d' % (len(emb_db[subj_name]) + 1)

        emb_db[subj_name][emb_name] = {'emb': emb, 'img': img}

    return emb_db


def create_embeddings_include_file(db_folder, db_filename, include_folder):
    """Converts binary embedding to a .h file to compile as a C code.
    """
    db_path = os.path.join(db_folder, db_filename + '.bin')
    data_bin = bytearray()

    with open(db_path, "rb") as file:
        S = int.from_bytes(file.read(1), byteorder='big', signed=False) # pylint: disable=invalid-name
        L = int.from_bytes(file.read(2), byteorder='big', signed=False) # pylint: disable=invalid-name
        N = int.from_bytes(file.read(2), byteorder='big', signed=False) # pylint: disable=invalid-name
        W = int.from_bytes(file.read(2), byteorder='big', signed=False) # pylint: disable=invalid-name
        H = int.from_bytes(file.read(2), byteorder='big', signed=False) # pylint: disable=invalid-name
        K = int.from_bytes(file.read(2), byteorder='big', signed=False) # pylint: disable=invalid-name

        subject_names_str = file.read(K).decode('ascii')
        subject_names_list = subject_names_str.split('  ')
        subject_names_str = '\0'.join(subject_names_list) + '\0'

        names_bytes = bytearray()
        names_bytes.extend(map(ord, subject_names_str))

        K = len(subject_names_str) # pylint: disable=invalid-name

        data_bin.extend(S.to_bytes(1, 'little', signed=False))
        data_bin.extend(L.to_bytes(2, 'little', signed=False))
        data_bin.extend(N.to_bytes(2, 'little', signed=False))
        data_bin.extend(W.to_bytes(2, 'little', signed=False))
        data_bin.extend(H.to_bytes(2, 'little', signed=False))
        data_bin.extend(K.to_bytes(2, 'little', signed=False))

        data_bin.extend(names_bytes)

        for _ in range((L+1)*N):
            next_d = file.read(1)
            data_bin.extend(next_d)

    data_arr = []
    data_line = []
    for next_d in data_bin:
        data_line.append(f'0x{next_d:02x}')
        if (len(data_line) % 18) == 0:
            data_arr.append(','.join(data_line))
            data_line.clear()

    data_arr.append(','.join(data_line))
    data = ', \\\n  '.join(data_arr)

    db_h_path = os.path.join(include_folder, db_filename + '.h')
    with open(db_h_path, 'w') as h_file:
        h_file.write('#define EMBEDDINGS { \\\n  ')
        h_file.write(data)
        h_file.write(' \\\n}')

    print(f'Embedding file is saved to {db_h_path}')
