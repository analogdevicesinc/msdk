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
import errno
import os
import argparse
import soundfile as sf
import numpy as np
import librosa

THRESHOLD = 30  # threshold to detect the beginning on an utterance


def resample(folder_in, folder_out, sr=16384):
    """
    Detects utterances in the input audio files and creates 1-sec 16khz
    mono .wav files as needed for KWS dataset
    """

    # create output folder
    try:
        os.mkdir(folder_out)
    except OSError as e:
        if e.errno == errno.EEXIST:
            pass
        else:
            raise
        print(f'{folder_out} already exists. overwrite!')

    for (dirpath, _, filenames) in os.walk(folder_in):
        for filename in sorted(filenames):

            file_cnt = 0
            i = 0
            if filename.endswith('.wav') or filename.endswith('.ogg'):
                fname = os.path.join(dirpath, filename)
                data, samplerate = librosa.load(fname, sr=sr)
                print(f'\rProcessing {fname}, sample rate={samplerate}', end="   ")
                mx = np.amax(abs(data))
                data = data/mx
                chunk_start = 0
                segment_len = 98*128

                while True:
                    if chunk_start + segment_len > len(data):
                        break

                    chunk = data[chunk_start: chunk_start+128]
                    avg = 1000*np.average(abs(chunk))

                    # visualize:
                    # bars = "=" * int(100 * avg/100)
                    # peak = avg * 100
                    # print("%04d %05d %s" % (i, peak, bars))

                    i += 128
                    if avg > THRESHOLD and chunk_start >= 30*128:
                        frame = data[chunk_start - 30*128:chunk_start + 98*128]
                        outfile = os.path.join(folder_out, filename[:-4] + str(file_cnt) + ".wav")
                        sf.write(outfile, frame, sr)
                        file_cnt += 1
                        chunk_start += 98*128
                    else:
                        chunk_start += 128
            else:
                continue
    print('\r')


def command_parser():
    """
    Return the argument parser
    """
    parser = argparse.ArgumentParser(description='Audio recorder command parser')
    parser.add_argument('-i', '--input', type=str, default='InputFolder', required=False,
                        help='input folder with audio files')

    parser.add_argument('-o', '--output', type=str, required=False, default='OutputWav',
                        help='output folder for segmented and resampled audio files')
    return parser.parse_args()


if __name__ == "__main__":
    command = command_parser()
    resample(command.input, command.output)
