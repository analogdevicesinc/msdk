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
Part number and device type conversion
"""
import argparse


def device(astring):
    """
    Take die type, or part number, and return the die type.
    """
    s = astring.lower()

    if s.startswith('max'):
        s = s[3:]  # Strip 'MAX' from part number
    elif s.startswith('ai'):
        s = s[2:]  # Strip 'AI' from die type

    try:
        num = int(s)
    except ValueError as exc:
        raise argparse.ArgumentTypeError(astring, 'is not a supported device type') from exc
    if num in [84, 85, 87]:  # Die types
        dev = num
    elif num == 78000:  # Part numbers
        dev = 85
    elif num == 78002:
        dev = 87
    else:
        raise argparse.ArgumentTypeError(astring, 'is not a supported device type')

    return dev


def partnum(num):
    """
    Return part number for a die type.
    """
    if num == 84:
        return 'AI84'
    if num == 85:
        return 'MAX78000'
    if num == 87:
        return 'MAX78002'

    raise RuntimeError(f'Unknown die type {num}')
