import numpy as np
import crc8
import serial
import time


def get_indata(data_path):
    with open(data_path, 'r') as f:
        data = f.readlines()
    buf = []
    cnt = 0
    for line in data:
        splitted_line = line.split(',')
        for el in splitted_line:
            el = el.replace(" ", "")
            if el.startswith('0x'):
                buf[-1].append(el[:10])
            elif el.startswith('#define'):
                buf.append([])
    return buf


def get_outdata(data_path):
    segment_len_idx = 3
    segment_len = 0
    segment = []
    segment_list = []

    with open(data_path) as f:
        while True:
            line = f.readline()
            if not line:
                segment_list.append(segment)
                segment = []
                break
            if line[0] != "#" and line[0] != "}":
                tmp = line[0 : len(line) - 1].rstrip()  # remove EOL
#                 tmp = tmp.replace("0x", "").replace(" ", "")  # remove 0x
                tmp = tmp.replace(" ", "") 
                tmp = tmp.rstrip(", \\")  # remove \
                tmp_list = tmp.rstrip(" \\").split(",")  # remove , at the end of line

                while tmp_list:
                    if segment_len == 0:
                        if len(tmp_list) >= segment_len_idx:
                            segment_len = int(tmp_list[segment_len_idx - 1], 16)
                            tmp_list = tmp_list[segment_len_idx:]
                            segment_len_idx = 3
                            segment_list.append(segment)
                            segment = []
                        else:
                            segment_len_idx -= len(tmp_list)
                            tmp_list = []

                    if segment_len >= len(tmp_list):
                        segment.extend(tmp_list)
                        segment_len -= len(tmp_list)
                        tmp_list = []
                    else:
                        segment.extend(tmp_list[:segment_len])
                        tmp_list = tmp_list[segment_len:]
                        segment_len = 0

    out_data = segment_list[1:]
    return out_data



def read_from_hex_list(buf, num_blocks, image_dim):
    inp = -1 * np.ones((4*num_blocks, image_dim, image_dim))
    idx = 0
    for x in range(image_dim):
        for y in range(image_dim):
            ind = image_dim*x + y
            for i in range(num_blocks):
                for j in range(4):
                    jj = 3-j
                    if jj != 3:
                        inp[i*4+j][x][y] = int(buf[i][ind][-8+(2*jj):-6+(2*jj)], 16)
                    else:
                        inp[i*4+j][x][y] = int(buf[i][ind][-2:], 16)
    return inp
    
    
def create_color_map(unfolded_inp, im_size):
    out_vals = np.argmax(unfolded_inp[:, :, :], axis=0)
    colors = np.zeros((im_size, im_size, 3), dtype=np.uint8)
    
    for ii, row in enumerate(out_vals):
        for j, val in enumerate(row):
            if val != 3:
                if val == 1:
                    val = 2
                elif val == 2:
                    val = 1
                colors[ii, j, val] = 255
                
    return colors
    

def fold_image(inp, fold_ratio):
    img_folded = None
    for i in range(fold_ratio):
        for j in range(fold_ratio):
            if img_folded is not None:
                img_folded = np.concatenate((img_folded, inp[i::fold_ratio, j::fold_ratio, :]), axis=2)
            else:
                img_folded = inp[i::fold_ratio, j::fold_ratio, :]

    return img_folded


def unfold_image(inp, fold_ratio, num_channels):
    if fold_ratio == 1:
        return inp
    dec0_u = np.zeros((num_channels, inp.shape[1]*fold_ratio, inp.shape[2]*fold_ratio))
    for i in range(fold_ratio):
        for j in range(fold_ratio):
            dec0_u[:, i::fold_ratio, j::fold_ratio] = inp[num_channels*(i*fold_ratio + j):num_channels*(i*fold_ratio + j + 1), :, :]
    return dec0_u


def read_output_from_txtfile(file_name):
    with open(file_name, 'r') as f:
        out = f.readlines()

    buf = []
    for i in range(len(out)):
        buf.append([])
        buf[-1].extend(out[i].split(',')[:-1])

    output = buf
    im_size = 88
    output = read_from_hex_list(output, 16, im_size)
    output = output.astype(np.int8)
    unfolded_inp = unfold_image(output, 4, 4)
    im_size = 352
    colors = create_color_map(unfolded_inp, im_size)
    return colors


def send_image_receive_output(ser, folded_image, output_filename):
    cnt = 0
    print('Please reset the device!')
    if (folded_image is None):
        print("image is captured from camera, result is displayed here!")
    else:
        while True:
            rx = ser.readline()
            rrx = rx[0:len(rx)-1].decode(errors="ignore")
            if rrx == "":
                continue
            print(f"\r\n>> {rrx}")
            if rrx.startswith("READY"):
                break

        for i_seg in range(12):
            current_segment = folded_image[i_seg*4:(i_seg+1)*4, :, :] - 128
            # print(current_segment.min(), current_segment.max())
            tmplst = []
            for i in range(current_segment.shape[1]):
                for j in range(current_segment.shape[2]):
                    hex_arr = ''
                    for ch in range(4):
                        jj = 3 - ch
                        hex_curr = hex(current_segment[jj, i, j])[2:]
                        if len(hex_curr) == 1:
                            hex_curr = '0' + hex_curr
                        hex_arr = ' '.join([hex_arr, hex_curr])

                    tmplst.append(hex_arr[1:])

            for b in tmplst:
                # send 4 bytes
                bb=bytes.fromhex(b)
                ser.write(bb)

                # send crc8
                hash = crc8.crc8()
                hash.update(bb)
                crc=hash.digest()
                ser.write(crc)

                cnt += 1

                # reading = ser.readline()
                print(f'\r{(100*cnt)//92928}%',end="")
                #time.sleep(0.001)

            time.sleep(0.1)

    print('\nWaiting for CNN Output from MAX78000 board')
    while True:
        ser.timeout = 0.1
        #rx = bytearray()
        #rx = ser.read(1)
        #print(f">> {rx}")
        rx = ser.readline()
        if len(rx) > 0: 
            rrx = rx[0:len(rx)-1].decode(errors="ignore")
            print(f">> {rrx}")

            if "SENDING_OUTPUT" in rrx:
                break

    rx_list = []
    print('\nGetting CNN Output\n')
    while len(rx_list) < 16:
        ser.timeout = 0.1
        rx = bytearray()
        while len(rx) < (4*88*88):
            temp = ser.read()
            if temp:
                rx.append(temp[0])
        rx_list.append(rx)
        print(f'\r{len(rx_list)}/16',end="")

    ser.close()

    print(f'\nSaving to the {output_filename}')

    with open(output_filename, "w") as f:
        for rx in rx_list:
            for i in range(len(rx)//4):
                tmp = rx[4*i:(4*i+4)]
                hex_num = '0x' + ''.join(format(x, '02x') for x in tmp[::-1])
                f.write(hex_num + ',')
            f.write('\n')        

    print('Output saved.')
                




    