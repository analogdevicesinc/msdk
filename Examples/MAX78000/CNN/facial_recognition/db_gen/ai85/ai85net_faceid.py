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
FaceID network for AI85/AI86

Optionally quantize/clamp activations
"""
from torch import nn

import ai85.ai8x as ai8x


class AI85FaceIDNet(nn.Module):
    """
    Simple FaceNet Model
    """
    def __init__(
            self,
            num_classes=None,  # pylint: disable=unused-argument
            num_channels=3,
            dimensions=(160, 120),  # pylint: disable=unused-argument
            bias=True,
            **kwargs
    ):
        super().__init__()

        self.conv1 = ai8x.FusedConv2dReLU(num_channels, 16, 3, padding=1,
                                          bias=False, **kwargs)
        self.conv2 = ai8x.FusedMaxPoolConv2dReLU(16, 32, 3, pool_size=2, pool_stride=2,
                                                 padding=1, bias=False, **kwargs)
        self.conv3 = ai8x.FusedMaxPoolConv2dReLU(32, 32, 3, pool_size=2, pool_stride=2,
                                                 padding=1, bias=bias, **kwargs)
        self.conv4 = ai8x.FusedMaxPoolConv2dReLU(32, 64, 3, pool_size=2, pool_stride=2,
                                                 padding=1, bias=bias, **kwargs)
        self.conv5 = ai8x.FusedMaxPoolConv2dReLU(64, 64, 3, pool_size=2, pool_stride=2,
                                                 padding=1, bias=bias, **kwargs)
        self.conv6 = ai8x.FusedConv2dReLU(64, 64, 3, padding=1, bias=bias, **kwargs)
        self.conv7 = ai8x.FusedConv2dReLU(64, 64, 3, padding=1, bias=bias, **kwargs)
        self.conv8 = ai8x.FusedMaxPoolConv2d(64, 512, 1, pool_size=2, pool_stride=2,
                                             padding=0, bias=False, **kwargs)
        self.avgpool = ai8x.AvgPool2d((5, 3), stride=1)

    def forward(self, x):  # pylint: disable=arguments-differ
        """Forward prop"""
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


def ai85faceidnet(pretrained=False, **kwargs):
    """
    Constructs a FaceIDNet model.
    """
    assert not pretrained
    return AI85FaceIDNet(**kwargs)


models = [
    {
        'name': 'ai85faceidnet',
        'min_input': 1,
        'dim': 3,
    },
]
