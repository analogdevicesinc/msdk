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
"""Contains adapter implementations for MAX78000 EvKit to get CNN model output.
"""

from .ai85_simulator import Simulator #pylint: disable=relative-beyond-top-level


class AI85Adapter:
    """
    Adapter base class for MAX78000 devices to get network output.
    """
    simulator = None
    def __init__(self):
        pass

    def get_network_out(self, data):
        """Returns output of the neural network on device."""

    def __del__(self):
        pass


class AI85SimulatorAdapter(AI85Adapter):
    """
    Adapter for MAX78000 simulator.
    """
    def __init__(self, path_to_checkpoint):
        super().__init__()
        self.simulator = Simulator(path_to_checkpoint)

    def get_network_out(self, data):
        """Returns output of the neural network on device."""
        return self.simulator.get_model_out(data)

    def __del__(self):
        if self.simulator is not None:
            del self.simulator
