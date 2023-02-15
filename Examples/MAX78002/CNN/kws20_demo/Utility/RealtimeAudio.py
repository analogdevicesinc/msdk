#!/usr/bin/env python3
###################################################################################################
################################################################################
 # Copyright (C) 2023 Maxim Integrated Products, Inc., All Rights Reserved.
 # 
 # This software is protected by copyright laws of the United States and
 # of foreign countries. This material may also be protected by patent laws
 # and technology transfer regulations of the United States and of foreign
 # countries. This software is furnished under a license agreement and/or a
 # nondisclosure agreement and may only be used or reproduced in accordance
 # with the terms of those agreements. Dissemination of this information to
 # any party or parties not specified in the license agreement and/or
 # nondisclosure agreement is expressly prohibited.
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
Utility to load a wave file and convert to headerfile, detect silence and keywords, split into 16K
samples and display waveform
"""
import argparse
import wave
import pyaudio
import numpy as np
import matplotlib.pyplot as plt


CHUNK = 128  # number of data points to read at a time and average for threshold
RATE = 16384  # Sampling rate
WINDOW = 16384  # number of samples to collect and send to AI85
THRESHOLD = 130  # voice detection threshold
YLIM = 10000  # Max Y value in Plots
PREAMBLE = 20 * CHUNK  # how many samples before beginning of keyword


def audio_capture(filename='', plotenable=False, duration=10, outputfilename='', eight=False):
    """
    Captures audio from wav file, converts to .h file, finds silence and keywords and
    plot audio file and splited keywords.
    """
    p = pyaudio.PyAudio()  # start the PyAudio class
    sampleCount = 0
    if filename:
        print("Reading from file:" + filename + "\r\n")
        wavefile = wave.open(filename, 'rb')
        stream = p.open(format=p.get_format_from_width(wavefile.getsampwidth()),
                        channels=wavefile.getnchannels(),
                        rate=wavefile.getframerate(),
                        output=True)
        duration = 10000  # large duration for file to go to the end of file
    else:
        print("Recording from Mic \r\n")
        stream = p.open(format=pyaudio.paInt16, channels=1, rate=RATE, input=True,
                        frames_per_buffer=CHUNK)

    # if an ouput header file needs to be generated
    if outputfilename:
        wr = open(command.output, 'w+')
        wr.write('#ifndef KWS20_TEST_VECTOR_H\n')
        wr.write('#define KWS20_TEST_VECTOR_H\n\n')
        if eight:
            wr.write('#define EIGHT_BIT_TEST_VECTOR\n\n')
        wr.write('#define KWS20_TEST_VECTOR {\\\n')

    # create a numpy array holding a single read of audio data
    y = np.array([])
    w = np.empty(PREAMBLE)
    ai85data = np.zeros((1, RATE))
    process = False
    word_count = 0
    for i in range(int(duration * RATE / CHUNK)):  # number of CHUNKs for duration

        if filename:
            data = np.fromstring(wavefile.readframes(CHUNK), dtype=np.int16)
        else:
            data = np.fromstring(stream.read(CHUNK), dtype=np.int16)
        # data = np.linspace(CHUNK*i,CHUNK*(i+1)-1,CHUNK)

        if data.size < CHUNK:
            break
        # print(data.shape)

        # add samples to a header file
        if outputfilename:
            for point in data:
                if eight:
                    point = int(point/256)
                wr.write('%d,\\\n' % point)
                sampleCount += 1

        avg = np.average(np.abs(data))

        # accumulate last chunk
        if (not process) and (avg < THRESHOLD):
            # print(w[CHUNK:].shape)
            # print(data.shape)
            w = np.append(w[CHUNK:], data)

        if (not process) and (avg >= THRESHOLD):
            process = True

        # start reading data
        if process:
            w = np.append(w, data)
            if w.size >= WINDOW:
                process = False
                ww = np.append(w[:WINDOW], np.zeros((1, RATE-WINDOW)))
                print(ww.shape)
                print(ww.size)
                print(ai85data.shape)

                print('+++++++++++++++++++++++++Size of W:  ', w.size)
                print(w.shape)
                ai85data = np.vstack([ai85data, ww])
                word_count += 1
                w = w[-PREAMBLE:]

                # add code to send to AI85

        bars = "=" * int(100 * avg / 1000)
        peak = avg
        print("%04d %05d %s" % (i, peak, bars))
        y = np.append(y, data)

    # close the stream gracefully
    stream.stop_stream()
    stream.close()
    p.terminate()

    print("count:", word_count)

    # row 0 is zero
    ai85data = np.delete(ai85data, 0, axis=0)
    print(ai85data.shape)
    # end created header file
    if outputfilename:
        # add zeros to the end to make it 16384
        if sampleCount < WINDOW:
            for i in range(WINDOW-sampleCount):
                wr.write('0,\\\n')
                sampleCount += 1
        wr.write('} \n')
        wr.write('#define KWS20_TEST_VECTOR_SIZE ')
        wr.write(sampleCount.__str__())
        wr.write('\n')
        wr.write('#endif \n')
        wr.close()

    if not plotenable:
        return

    # plot data
    x = range(y.size)
    if word_count == 0:
        word_count = 1  # to plot the main one at least

    grid = plt.GridSpec(2, word_count, wspace=0.2, hspace=0.2)

    # complete waveform
    plt.subplot(grid[0, :])
    plt.title('Complete Waveform')
    plt.ylim([-YLIM, YLIM])
    plt.grid(True)
    plt.grid(color='b', ls='-.', which='both', lw=0.25, animated=True)
    plt.plot(x, y, color='red')

    # individual plots if there is any keyword data captured
    if ai85data.shape[0] > 0:
        for i in range(0, word_count):
            plt.subplot(grid[1, i])
            plt.title('#:' + (i + 1).__str__())
            plt.ylim([-YLIM, YLIM])
            plt.plot(range(RATE), ai85data[i, :])
            # print(ai85data[i, :].shape)
    plt.show()


def command_parser():
    """
    Return the argument parser
    """
    parser = argparse.ArgumentParser(description='Audio recorder command parser')
    parser.add_argument('-i', '--input', type=str, default='',
                        help='input wave file name')
    parser.add_argument('-p', '--plot', action="store_true", help='Plot waves',
                        default=False)
    parser.add_argument('-d', '--duration', type=int, default=10,
                        help='Duration (sec)')
    parser.add_argument('-o', '--output', type=str, default='',
                        help='output .h file from wav')
    parser.add_argument('-8', '--eight', action="store_true", default=False,
                        help='8 bit header file')
    return parser.parse_args()


if __name__ == "__main__":
    command = command_parser()
    audio_capture(command.input, command.plot, command.duration, command.output, command.eight)
