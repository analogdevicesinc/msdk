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
