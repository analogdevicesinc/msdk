# Copyright 2021 Open Source Robotics Foundation, Inc.
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

import filecmp
import pathlib

from rosidl_cli.command.translate.api import translate


DATA_PATH = pathlib.Path(__file__).parent / 'data'


def test_idl2ddsidl_translation(tmp_path, capsys):
    # NOTE(hidmic): pytest and empy do not play along,
    # the latter expects some proxy will stay in sys.stdout
    # and the former insists in overwriting it

    with capsys.disabled():
        # Test .idl to _.idl translation
        idl_files = translate(
            package_name='test_msgs',
            interface_files=[
                f'{DATA_PATH}:msg/Test.idl'],
            output_path=tmp_path,
            output_format='dds_idl',
            translators=['idl2ddsidl']
        )

        assert len(idl_files) == 1
        idl_file = idl_files[0]
        assert idl_file == f'{tmp_path}:msg/Test_.idl'
        assert filecmp.cmp(
            tmp_path / 'msg' / 'Test_.idl',
            DATA_PATH / 'msg' / 'Test_.expected.idl',
            shallow=False
        )
