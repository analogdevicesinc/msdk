copy_right = '''
/******************************************************************************
 * Copyright (C) 2023 Maxim Integrated Products, Inc., All Rights Reserved.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a
 * copy of this software and associated documentation files (the "Software"),
 * to deal in the Software without restriction, including without limitation
 * the rights to use, copy, modify, merge, publish, distribute, sublicense,
 * and/or sell copies of the Software, and to permit persons to whom the
 * Software is furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included
 * in all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS
 * OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.
 * IN NO EVENT SHALL MAXIM INTEGRATED BE LIABLE FOR ANY CLAIM, DAMAGES
 * OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE,
 * ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR
 * OTHER DEALINGS IN THE SOFTWARE.
 *
 * Except as contained in this notice, the name of Maxim Integrated
 * Products, Inc. shall not be used except as stated in the Maxim Integrated
 * Products, Inc. Branding Policy.
 *
 * The mere transfer of this software does not imply any licenses
 * of trade secrets, proprietary technology, copyrights, patents,
 * trademarks, maskwork rights, or any other form of intellectual
 * property whatsoever. Maxim Integrated Products, Inc. retains all
 * ownership rights.
 *
 ******************************************************************************/
'''

import sys
import os
import re


def get_fullpath(file_dir, file_name):
    if file_dir == "":
        return file_name
    if os.name == "posix":
        return file_dir + '/' + file_name
    if os.name == "nt":
        return file_dir + '\\' + file_name


def parse_scpcmd_file(filename):
    f = open(filename, "r")
    packet_list_file = f.read()
    f.close()
    file_dir = os.path.dirname(filename)

    packets = []

    # Get number of packets to send
    for line in packet_list_file.split("\n"):
        file_name = line.strip()
        if file_name == '':
            continue

        s_m = re.search('(\w+[_-]*\w+)\.(\d+)\.(\w+[_-]*\w+)\.((\w+[_-]*)*)\.\w+', file_name)
        if s_m is not None:
            id = s_m.group(2)
            cmd = s_m.group(4)
            way_str = s_m.group(3)
        else:
            print("error: wrong filename: " + file_name)
            raise

        if way_str == 'bl':
            is_tx = 0 # bl will send this to the host
        elif way_str == 'host':
            is_tx = 1 # host will send this to target
        else:
            print("error: wrong filename: " + file_name)
            raise

        # read packet data
        data_str = ''
        with open(get_fullpath(file_dir, file_name), 'rb') as f:
            data_bin = f.read()

            for i, x in enumerate(data_bin):
                if (i != 0) and ((i % 16) == 0):
                    data_str += '\n'
                data_str += '0x{:02x}, '.format(x)
            data_str = data_str[:-2] # remove last comma

        if data_str is None:
            print("Error : Unable to read file packet : " + file_dir)
            raise

        if cmd in ("hello_request",):
            packet_type = 1
        elif cmd in ("hello_reply",):
            packet_type = 2
        elif cmd in ("erase_mem", "del_mem"):
            packet_type = 3
        elif cmd in ("erase_mem_response", "del_mem_response"):
            packet_type = 4
        elif cmd in ("write_mem",):
            packet_type = 5   
        elif cmd in ("write_mem_response",):
            packet_type = 6   
        elif cmd in ("dump",):
            packet_type = 7
        elif cmd in ("write_crk_response",):
            packet_type = 8     
        else:
            packet_type = 0

        arr_name = f'scp_{id}_{cmd}'
        arr_len = len(data_bin)
        packets.append((packet_type, is_tx, arr_name, arr_len, data_str))

    return packets


if __name__ == "__main__":

    if len(sys.argv) > 3:
        packet_list = sys.argv[1]
        chip = sys.argv[2]
        image = sys.argv[3]
    elif len(sys.argv) > 2:
        packet_list = sys.argv[1]
        chip = sys.argv[2]
        image = 'fw'
    elif len(sys.argv) > 1:
        packet_list = sys.argv[1]
        chip = 'MAX32520KIT'
        image = 'fw'
    else:
        print('Usage error, please pass packet.list file as parameter')
        exit(-1)

    packets_data = parse_scpcmd_file(packet_list)

    target_file_name = f'scp_{chip}_{image}.c'
    with open(target_file_name, 'w') as f:
        f.write(copy_right)
        f.write('\n\n')

        scp_packets_arr = f'const scp_packet_struct scp_{chip}_{image}[] = {{ \n'
        for t, d, name, l, data in packets_data:
            f.write(f'static const unsigned char {name}[] = {{ {data} }};')
            f.write('\n\n')

            scp_packets_arr += f'{{ {t:<2}, {d:<2}, {l:<8}, {name} }},\n'

        scp_packets_arr += f'{{ {0:<2}, {0:<2}, {0:<8}, 0 }} \n' # to demonstrate end of packet
        scp_packets_arr += '};'  # end of array

        f.write('''
typedef struct {
	unsigned char type; // 1:hello_reply, 2:erase/del_mem
    unsigned char is_tx;// 1: From host to target, 0: From target to host
    unsigned short len;
    const unsigned char *data;
} scp_packet_struct;\n\n
''')

        f.write(scp_packets_arr)
        f.write('\n')
        print(target_file_name + " generated.")
