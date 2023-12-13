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
"""
FaceID network for AI85/AI86

Optionally quantize/clamp activations
"""
import torch.nn as nn
import ai85.ai8x as ai8x


class AI85FaceIDNetNoBias(nn.Module):
    """
    Simple FaceNet Model
    """
    def __init__(
            self,
            num_classes=None,  # pylint: disable=unused-argument
            num_channels=3,
            dimensions=(160, 120),  # pylint: disable=unused-argument
            bias=False # pylint: disable=unused-argument
    ):
        super(AI85FaceIDNetNoBias, self).__init__()

        self.conv1 = ai8x.FusedConv2dReLU(num_channels, 16, 3, padding=1,
                                          bias=False)
        self.conv2 = ai8x.FusedMaxPoolConv2dReLU(16, 32, 3, pool_size=2, pool_stride=2,
                                                 padding=1, bias=False)
        self.conv3 = ai8x.FusedMaxPoolConv2dReLU(32, 32, 3, pool_size=2, pool_stride=2,
                                                 padding=1, bias=False)
        self.conv4 = ai8x.FusedMaxPoolConv2dReLU(32, 64, 3, pool_size=2, pool_stride=2,
                                                 padding=1, bias=False)
        self.conv5 = ai8x.FusedMaxPoolConv2dReLU(64, 64, 3, pool_size=2, pool_stride=2,
                                                 padding=1, bias=False)
        self.conv6 = ai8x.FusedConv2dReLU(64, 64, 3, padding=1, bias=False)
        self.conv7 = ai8x.FusedConv2dReLU(64, 64, 3, padding=1, bias=False)
        self.conv8 = ai8x.FusedMaxPoolConv2d(64, 512, 1, pool_size=2, pool_stride=2,
                                             padding=0, bias=False)
        self.avgpool = ai8x.AvgPool2d((5, 3))

    def forward(self, x):  # pylint: disable=arguments-differ
        x = self.conv1(x)
        x = self.conv2(x)
        x = self.conv3(x)
        x = self.conv4(x)
        x = self.conv5(x)
        x = self.conv6(x)
        x = self.conv7(x)
        x = self.conv8(x)
        x = self.avgpool(x)
        return x


def ai85faceidnetnobias(pretrained=False, **kwargs):
    """
    Constructs a FaceIDNet model.
    """
    assert not pretrained
    return AI85FaceIDNetNoBias(**kwargs)


models = [
    {
        'name': 'ai85faceidnetnobias',
        'min_input': 1,
        'dim': 3,
    },
]
