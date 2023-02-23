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
Voice recorder
"""
import wave
import argparse
import pyaudio


SR = 16000  # sample per sec
CHANNEL = 1  # number of input channel
FORMAT = pyaudio.paInt16  # data format
CHUNK = 1024
LENGTH = 4


def audio_recorder(filename='output.wav', recordlength=LENGTH, samplerate=SR):
    """
    Record audio to a file.
    """
    audio = pyaudio.PyAudio()

    print("Recording started for", recordlength, " sec")
    stream = audio.open(format=FORMAT,
                        channels=CHANNEL,
                        rate=samplerate,
                        input=True,
                        frames_per_buffer=CHUNK)

    frame = []
    for i in range(0, int(samplerate / CHUNK * recordlength)):
        data = stream.read(CHUNK)
        frame.append(data)
        print(i)

    print("Recording finished!")
    stream.stop_stream()
    stream.close()
    audio.terminate()

    # Store in file
    wf = wave.open(filename, 'wb')
    wf.setnchannels(CHANNEL)
    wf.setsampwidth(audio.get_sample_size(FORMAT))
    wf.setframerate(samplerate)
    wf.writeframes(b''.join(frame))
    wf.close()


def command_parser():
    """
    Return the argument parser
    """
    parser = argparse.ArgumentParser(description='Audio recorder command parser')
    parser.add_argument('-d', '--duration', type=int, default=LENGTH,
                        help='audio recording duration (default:' + LENGTH.__str__() + ')')
    parser.add_argument('-sr', '--samplerate', type=int, default=SR,
                        help='recording samplerate (default:' + SR.__str__() + ')')
    parser.add_argument('-o', '--output', type=str, default='voice.wav',
                        help='output wavefile name')
    return parser.parse_args()


if __name__ == "__main__":
    command = command_parser()
    print("Output Name = ", command.output)
    print("Sample Rate = ", command.samplerate)
    print("Duration = ", command.duration)
    audio_recorder(command.output, command.duration, command.samplerate)
