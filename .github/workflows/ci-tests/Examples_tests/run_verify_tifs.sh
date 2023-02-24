#!/bin/bash

python3 /home/$USER/Workspace/Resource_Share/Resource_Share.py -l -t 3600 max32655_board1
python3 /home/$USER/Workspace/Resource_Share/Resource_Share.py -l -t 3600 max32655_board2


GITHUB_WORKSPACE=/home/btm-ci/Workspace/btm-ci-github-runner2/_work/msdk/msdk

bash /home/$USER/Workspace/BLE-examples-test/Examples_tests/verify_tifs.sh $GITHUB_WORKSPACE

python3 /home/$USER/Workspace/Resource_Share/Resource_Share.py max32655_board1
python3 /home/$USER/Workspace/Resource_Share/Resource_Share.py max32655_board2
