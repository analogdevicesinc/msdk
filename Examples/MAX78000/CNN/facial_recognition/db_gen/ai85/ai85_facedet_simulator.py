###############################################################################
 #
 # Copyright (C) 2022-2023 Maxim Integrated Products, Inc. (now owned by
 # Analog Devices, Inc.),
 # 2023 Analog Devices, Inc. All Rights Reserved.
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
from .ai85net_tinierssd_face import TinierSSDFace
import ai85.ai8x as ai8x
from .ai8x import set_device #pylint: disable=relative-beyond-top-level

class Simulator:
    """
    MAX78000 Simulator.
    """
    model = None

    def __init__(self, checkpoint_path):
        self.device = self.__get_device()
        #load model
        set_device(85, False, False)
        self.model = TinierSSDFace(num_classes = 2, device = self.device)
        qat_policy = parse_qat_yaml.parse("db_gen/model/qat_policy_face.yaml")
        ai8x.fuse_bn_layers(self.model)
        ai8x.initiate_qat(self.model, qat_policy)
        checkpoint = torch.load(checkpoint_path, map_location=self.device)
        self.model.load_state_dict(checkpoint['state_dict'])
        ai8x.update_model(self.model)
        self.model = self.model.to(self.device)


    def __get_device(self): #pylint: disable=no-self-use
        device = torch.device('cuda:0' if torch.cuda.is_available() else 'cpu')
        print('Running on device: {}'.format(device))
        return device
    

    def get_model_out(self, data_in):
        """Returns output of the neural network on device.""" 
        set_device(85, False, False, verbose=False)
        locs, scores = self.model(data_in)
        return locs, scores

    def __del__(self):
        if self.model is not None:
            del self.model
        if self.device.type != 'cpu':
            torch.cuda.empty_cache()
    
    
