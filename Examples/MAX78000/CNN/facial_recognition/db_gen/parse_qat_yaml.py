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
"""
Parses YAML file used to define Quantization Aware Training
"""

import yaml


def parse(yaml_file):
    """
    Parses `yaml_file` that defines the QAT policy
    """
    policy = None
    with open(yaml_file, mode='r', encoding='utf-8') as stream:
        try:
            policy = yaml.safe_load(stream)
        except yaml.YAMLError as exc:
            print(exc)

    print(policy)

    if policy and 'start_epoch' not in policy:
        assert False, '`start_epoch` must be defined in QAT policy'
    if policy and 'weight_bits' not in policy:
        assert False, '`weight_bits` must be defined in QAT policy'

    return policy
