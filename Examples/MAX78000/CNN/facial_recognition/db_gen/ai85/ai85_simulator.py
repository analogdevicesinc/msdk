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
"""Contains MAX78000 simulator implementations to get CNN model output.
"""
import numpy as np
import torch

import parse_qat_yaml
import ai85.ai85net_faceid_112 as ai85netfaceidnet_112
import ai85.ai8x as ai8x
from .ai8x import set_device #pylint: disable=relative-beyond-top-level
import torchvision
from distiller import apputils
class Simulator:
    """
    MAX78000 Simulator.
    """
    model = None

    def __init__(self, checkpoint_path):
        self.device = self.__get_device()
        #load model
        set_device(85, True, True)
        self.model = ai85netfaceidnet_112.ai85faceidnet_112(bias = True, num_classes=None).to(self.device)
        ai8x.fuse_bn_layers(self.model)
        #checkpoint = torch.load(checkpoint_path, map_location=self.device)
        self.model = apputils.load_lean_checkpoint(self.model, checkpoint_path, model_device=self.device)

        #self.model.load_state_dict(checkpoint['state_dict'])
        ai8x.update_model(self.model)
        self.model = self.model.to(self.device)
        self.model.eval()

    def __get_device(self): #pylint: disable=no-self-use
        device = torch.device('cuda:0' if torch.cuda.is_available() else 'cpu')
        print('Running on device: {}'.format(device))
        return device

    def get_model_out(self, data_in):
        """Returns output of the neural network on device."""
        set_device(85, True, True, verbose=False)
        
        in_tensor = data_in.transpose([2, 0, 1])[None, ].copy()
        if in_tensor.dtype == np.uint8:
            in_tensor = in_tensor.astype(np.float32)
            in_tensor -= 128
        else:
            in_tensor = in_tensor.astype(np.float32)
            in_tensor = 255 * in_tensor - in_tensor.min() / (in_tensor.max() - in_tensor.min())
            in_tensor = np.round(in_tensor) - 128
        in_tensor = torch.from_numpy(in_tensor).to(self.device)
        #print(in_tensor.min(), in_tensor.max(), in_tensor.shape)
        data_out = self.model(in_tensor)
        return data_out.detach().cpu().numpy()

    def __del__(self):
        if self.model is not None:
            del self.model
        if self.device.type != 'cpu':
            torch.cuda.empty_cache()
